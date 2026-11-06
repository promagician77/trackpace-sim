"""Fly laps of a 400 m track at a programmed pace with PX4 offboard control (MAVSDK).

    python -m trackpace.fly config/pace_59.toml --out runs/ff59

Every setpoint is computed from the pace schedule on the autopilot's clock, so lap
timing does not depend on how fast this script runs: a late tick sends the setpoint
for the true current time, not the next one in a queue.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import shutil
import sys
import time
from collections import deque
from pathlib import Path

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.offboard import OffboardAsync
from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
from mavsdk.plugins.offboard import AccelerationNed, PositionNedYaw, VelocityNedYaw

from . import config as cfgmod
from .schedule import PacePlan
from .track import Placement, Track


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class AutopilotClock:
    """Autopilot time (us since boot) from ODOMETRY time_usec.

    Offset = min(local receive time - autopilot stamp) over a sliding window: the
    least-delayed message bounds the transport delay and follows any drift between
    the simulator clock and wall time.
    """

    def __init__(self, window: int = 100):
        self.offsets: deque[float] = deque(maxlen=window)
        self.ned = (math.nan, math.nan, math.nan)
        self.stamp_us = 0

    def update(self, time_usec: int, n: float, e: float, d: float) -> None:
        self.offsets.append(time.monotonic() * 1e6 - time_usec)
        self.ned = (n, e, d)
        self.stamp_us = time_usec

    @property
    def ready(self) -> bool:
        return len(self.offsets) >= 10

    def now_us(self) -> float:
        return time.monotonic() * 1e6 - min(self.offsets)


def setpoint(track: Track, place: Placement, offset: float, alt: float, s: float, v: float, a: float,
             drag: float = 0.0):
    """NED position, velocity, acceleration and yaw for the drone beside a runner at s.

    drag (1/s): identified linear drag per unit mass (trackpace.identify). Adding drag*v to
    the acceleration feed-forward gives PX4 the tilt needed to hold speed, so its velocity
    integrator is not left to discover it lap after lap, bend after bend.
    """
    x, y, p = track.offset_point(s, offset)
    k = 1.0 + offset * p.kappa                      # drone path / runner path length ratio
    w = v * k                                      # drone speed
    vx, vy = p.tx * w, p.ty * w
    # tangential + centripetal (towards the bend centre = runner's left normal)
    kd = p.kappa / k if p.kappa else 0.0           # curvature of the offset path
    ax = p.tx * a * k + (-p.ty) * w * w * kd + drag * vx
    ay = p.ty * a * k + (p.tx) * w * w * kd + drag * vy
    n, e = place.to_ned(x, y)
    vn, ve = place.vec_to_ned(vx, vy)
    an, ae = place.vec_to_ned(ax, ay)
    yaw = place.yaw_deg(p.tx, p.ty)
    return (n, e, -alt), (vn, ve, 0.0), (an, ae, 0.0), yaw


async def wait_for(gen, pred, timeout: float, what: str):
    async def _w():
        async for x in gen:
            if pred(x):
                return x
    try:
        return await asyncio.wait_for(_w(), timeout)
    except asyncio.TimeoutError:
        raise SystemExit(f"timed out waiting for {what}")


async def run(cfg: dict, out: Path) -> dict:
    tcfg, pcfg, ccfg = cfg["track"], cfg["pace"], cfg["control"]
    track = Track(lane=int(tcfg["lane"]))
    plan = PacePlan(pcfg["laps_s"], track.length, pcfg["runup_accel"], pcfg["runup_max_m"],
                    pcfg["ramp_accel"], pcfg["stop_decel"])
    offset, alt, mode = float(tcfg["offset_m"]), float(tcfg["altitude_m"]), ccfg["mode"]
    drag = float(ccfg["drag_per_s"]) if mode == "ff" else 0.0
    out.mkdir(parents=True, exist_ok=True)

    conf = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    async with Mavsdk(conf) as mav:
        await mav.add_any_connection(cfg["connection"]["url"])
        system = await mav.first_autopilot(timeout_s=float(cfg["connection"]["timeout_s"]))
        if system is None:
            raise SystemExit("no autopilot found on " + cfg["connection"]["url"])
        tel, act, off, par = TelemetryAsync(system), ActionAsync(system), OffboardAsync(system), ParamAsync(system)
        log("autopilot connected")

        # parameters: set requested ones, record the ones that shape tracking
        for name, value in cfg["px4"].items():
            if isinstance(value, int) and not isinstance(value, bool):
                await par.set_param_int(name, value)
            else:
                await par.set_param_float(name, float(value))
            log(f"param {name} = {value}")
        params = {}
        for name in cfgmod.PARAMS_TO_RECORD:
            try:
                params[name] = round(await par.get_param_float(name), 4)
            except Exception:
                pass
        vmax = params.get("MPC_XY_VEL_MAX", 12.0)
        w_max = plan.max_speed * (1 + max(offset, 0.0) / track.R)
        if w_max > 0.95 * vmax:
            raise SystemExit(f"pace needs {w_max:.2f} m/s, MPC_XY_VEL_MAX is {vmax}; raise it first")

        clock = AutopilotClock()
        await tel.set_rate_odometry(50.0)

        async def pump():
            async for od in tel.subscribe_odometry():
                pb = od.position_body
                clock.update(od.time_usec, pb.x_m, pb.y_m, pb.z_m)
        pump_task = asyncio.create_task(pump())

        await wait_for(tel.subscribe_health_all_ok(), bool, 60, "health checks")
        while not clock.ready:
            await asyncio.sleep(0.05)
        log("health ok, clock locked")

        await act.set_takeoff_altitude(alt)
        # a parameter change (e.g. EKF2_*) can make the estimator re-initialise: wait, retry
        for attempt in range(30):
            try:
                await act.arm()
                break
            except Exception as exc:
                if attempt == 29:
                    raise
                if attempt == 0:
                    log(f"arming refused ({exc}); waiting for health checks")
                await asyncio.sleep(1.0)
        await act.takeoff()
        log(f"takeoff to {alt} m")
        t_w = time.monotonic()
        while -clock.ned[2] < alt - 0.25:
            if time.monotonic() - t_w > 40:
                raise SystemExit("takeoff did not reach altitude")
            await asyncio.sleep(0.1)
        await asyncio.sleep(2.0)

        # place the track in the local frame
        n0, e0, _ = clock.ned
        s_start = plan.segs[0].s0
        if tcfg["anchor"] == "local":
            place = Placement(float(tcfg["centre_north"]), float(tcfg["centre_east"]), tcfg["bearing_deg"])
        elif tcfg["anchor"] == "gps":
            org = await tel.get_gps_global_origin()
            place = Placement.from_gps(tcfg["centre_lat"], tcfg["centre_lon"],
                                       org.latitude_deg, org.longitude_deg, tcfg["bearing_deg"])
        else:
            place = Placement.anchored_at(track, s_start, offset, n0, e0, tcfg["bearing_deg"])
        start_ned, _, _, start_yaw = setpoint(track, place, offset, alt, s_start, 0.0, 0.0)

        # offboard: hold, transit (gps/local anchor), turn to the straight
        await off.set_position_ned(PositionNedYaw(n0, e0, -alt, start_yaw))
        await off.start()
        log("offboard active")
        dist = math.hypot(start_ned[0] - n0, start_ned[1] - e0)
        if dist > 0.5:
            spd = float(ccfg["transit_speed"])
            t_tr = dist / spd
            t_a = time.monotonic()
            while (f := (time.monotonic() - t_a) / t_tr) < 1.0:
                await off.set_position_ned(PositionNedYaw(n0 + f * (start_ned[0] - n0),
                                                          e0 + f * (start_ned[1] - e0), -alt, start_yaw))
                await asyncio.sleep(0.05)
            log(f"transit {dist:.1f} m to run-up start")
        await off.set_position_ned(PositionNedYaw(start_ned[0], start_ned[1], -alt, start_yaw))

        # schedule zero = the planned line crossing, on the autopilot clock
        t0_us = clock.now_us() + (float(ccfg["hold_s"]) - plan.start) * 1e6
        log(f"go in {ccfg['hold_s']} s; laps {plan.laps}; line at AP t={t0_us/1e6:.3f} s")

        rows = []
        period = 1.0 / float(ccfg["rate_hz"])
        t_end = plan.end + float(ccfg["tail_s"])
        next_tick = time.monotonic()
        last_lap = -1
        late = 0
        while True:
            now_us = clock.now_us()
            t = (now_us - t0_us) / 1e6
            if t > t_end:
                break
            st = plan.at(t)
            pos, vel, acc, yaw = setpoint(track, place, offset, alt, st.s, st.v, st.a, drag)
            if mode == "ff":
                await off.set_position_velocity_acceleration_ned(
                    PositionNedYaw(*pos, yaw), VelocityNedYaw(*vel, yaw), AccelerationNed(*acc))
            elif mode == "pv":
                await off.set_position_velocity_ned(PositionNedYaw(*pos, yaw), VelocityNedYaw(*vel, yaw))
            else:
                await off.set_position_ned(PositionNedYaw(*pos, yaw))
            n, e, d = clock.ned
            rows.append((round(now_us), round(t, 4), round(st.s, 3), round(st.v, 3), st.kind,
                         *(round(x, 3) for x in pos), *(round(x, 3) for x in vel),
                         *(round(x, 3) for x in acc), round(yaw, 2),
                         clock.stamp_us, round(n, 3), round(e, 3), round(d, 3)))
            if st.lap != last_lap and st.lap > 0:
                log(f"lap {st.lap} ({plan.laps[st.lap-1]:.2f} s)")
                last_lap = st.lap
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                late += 1
                next_tick = time.monotonic()

        await off.set_position_ned(PositionNedYaw(*pos, yaw))
        await off.stop()
        await act.land()
        log("landing")
        await wait_for(tel.subscribe_in_air(), lambda x: not x, 60, "landing")
        await asyncio.sleep(3.0)                           # let auto-disarm close the log
        pump_task.cancel()

    hdr = ["ap_us", "t", "s_plan", "v_plan", "phase", "sp_n", "sp_e", "sp_d", "sv_n", "sv_e", "sv_d",
           "sa_n", "sa_e", "sa_d", "yaw", "od_us", "od_n", "od_e", "od_d"]
    with open(out / "setpoints.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(rows)
    run_info = {
        "mode": mode, "config": cfg, "params": params, "plan": plan.summary(),
        "t0_ap_us": t0_us, "track": {"lane": track.lane, "R": track.R, "L": track.L, "length": track.length},
        "placement": {"north0": place.north0, "east0": place.east0, "bearing_deg": place.bearing_deg},
        "offset_m": offset, "altitude_m": alt, "drag_per_s": drag, "rate_hz": ccfg["rate_hz"], "late_ticks": late,
        "ticks": len(rows), "finished_unix": time.time(),
    }
    (out / "run.json").write_text(json.dumps(run_info, indent=1))
    log(f"wrote {out}/run.json and setpoints.csv ({len(rows)} setpoints, {late} late ticks)")
    return run_info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", nargs="?", help="TOML pace config")
    ap.add_argument("--out", default="runs/latest", help="output folder")
    ap.add_argument("--mode", choices=["ff", "pv", "pos"], help="override control.mode")
    ap.add_argument("--lap", type=float, help="override: constant lap time (s)")
    ap.add_argument("--laps", type=int, help="override: number of laps")
    ap.add_argument("--url", help="override connection.url")
    ap.add_argument("--drag", type=float, help="override control.drag_per_s (1/s)")
    ap.add_argument("--ulog-dir", help="PX4 log folder (SITL: build/px4_sitl_default/rootfs/log); "
                    "the flight's .ulg is copied into --out")
    a = ap.parse_args(argv)
    cfg = cfgmod.load(a.config)
    if a.mode:
        cfg["control"]["mode"] = a.mode
    if a.url:
        cfg["connection"]["url"] = a.url
    if a.drag is not None:
        cfg["control"]["drag_per_s"] = a.drag
    if a.lap or a.laps:
        n = a.laps or len(cfg["pace"]["laps_s"])
        T = a.lap or cfg["pace"]["laps_s"][0]
        cfg["pace"]["laps_s"] = [float(T)] * n
    t_start = time.time()
    out = Path(a.out)
    info = asyncio.run(run(cfg, out))
    if a.ulog_dir:
        logs = [p for p in Path(a.ulog_dir).rglob("*.ulg") if p.stat().st_mtime >= t_start - 5]
        if logs:
            src = max(logs, key=lambda p: p.stat().st_mtime)
            shutil.copy2(src, out / "flight.ulg")
            info["ulog"] = str(out / "flight.ulg")
            info["ulog_source"] = src.name
            (out / "run.json").write_text(json.dumps(info, indent=1))
            log(f"copied {src.name} -> {out/'flight.ulg'}")
        else:
            log("no new .ulg found in " + a.ulog_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
