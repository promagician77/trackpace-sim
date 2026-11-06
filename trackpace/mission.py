"""Baseline: the same laps as a PX4 waypoint mission (what a QGroundControl plan would do).

    python -m trackpace.mission config/pace_59.toml --laps 2 --out runs/mission59

Waypoints follow the lane's measurement line (every 7.2 m on the bends, ends of the
straights), fly-through, mission speed = lap length / target lap time. Scored with the
same analysis, using the first line crossing as the start of timing.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import shutil
import sys
import time
from pathlib import Path

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.mission import MissionAsync
from mavsdk.asyncio.plugins.param import ParamAsync
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync
from mavsdk.plugins.mission import MissionItem, MissionPlan

from . import config as cfgmod
from .fly import AutopilotClock, log, wait_for
from .schedule import PacePlan
from .track import Placement, Track

R_EARTH = 6371000.0


def waypoints_s(track: Track, laps: int, runup: float, bend_step: float = 7.2, tail: float = 20.0) -> list[float]:
    """Progress values for waypoints: straights by their ends, bends every ~bend_step m."""
    nb = max(2, round(track.bend / bend_step))
    one = [0.0]
    one += [track.bend * i / nb for i in range(1, nb + 1)]                    # bend 1
    one += [track.back_end]                                                   # end of back straight
    one += [track.back_end + track.bend * i / nb for i in range(1, nb + 1)]   # bend 2
    # home straight ends at the line (next lap's 0)
    out = [-runup + 1.0]
    for k in range(laps):
        out += [k * track.length + s for s in one]
    out += [laps * track.length, laps * track.length + tail]
    return out


async def run(cfg: dict, out: Path, bend_step: float) -> dict:
    tcfg, pcfg = cfg["track"], cfg["pace"]
    track = Track(lane=int(tcfg["lane"]))
    laps = pcfg["laps_s"]
    plan = PacePlan(laps, track.length, pcfg["runup_accel"], pcfg["runup_max_m"],
                    pcfg["ramp_accel"], pcfg["stop_decel"])
    speed = track.length / laps[0]
    alt, offset = float(tcfg["altitude_m"]), float(tcfg["offset_m"])
    out.mkdir(parents=True, exist_ok=True)

    conf = Configuration.create_with_component_type(ComponentType.GROUND_STATION)
    async with Mavsdk(conf) as mav:
        await mav.add_any_connection(cfg["connection"]["url"])
        system = await mav.first_autopilot(timeout_s=float(cfg["connection"]["timeout_s"]))
        tel, act, mis, par = TelemetryAsync(system), ActionAsync(system), MissionAsync(system), ParamAsync(system)
        params = {}
        for name in cfgmod.PARAMS_TO_RECORD:
            try:
                params[name] = round(await par.get_param_float(name), 4)
            except Exception:
                pass
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

        await act.set_takeoff_altitude(alt)
        await act.arm()
        await act.takeoff()
        while -clock.ned[2] < alt - 0.25:
            await asyncio.sleep(0.1)
        await asyncio.sleep(2.0)
        n0, e0, _ = clock.ned
        if tcfg["anchor"] == "local":
            place = Placement(float(tcfg["centre_north"]), float(tcfg["centre_east"]), tcfg["bearing_deg"])
        else:
            place = Placement.anchored_at(track, -plan.runup_distance, offset, n0, e0, tcfg["bearing_deg"])
        org = await tel.get_gps_global_origin()
        lat0, lon0 = org.latitude_deg, org.longitude_deg

        items = []
        for s in waypoints_s(track, len(laps), plan.runup_distance, bend_step):
            x, y, _ = track.offset_point(s, offset)
            n, e = place.to_ned(x, y)
            lat = lat0 + math.degrees(n / R_EARTH)
            lon = lon0 + math.degrees(e / (R_EARTH * math.cos(math.radians(lat0))))
            items.append(MissionItem(lat, lon, alt, speed, True, float("nan"), float("nan"),
                                     MissionItem.CameraAction.NONE, float("nan"), float("nan"),
                                     float("nan"), float("nan"), float("nan"), MissionItem.VehicleAction.NONE))
        await mis.set_return_to_launch_after_mission(False)
        await mis.upload_mission(MissionPlan(items))
        log(f"mission uploaded: {len(items)} waypoints at {speed:.2f} m/s")
        # PX4 marks a freshly uploaded mission available a moment after the upload
        # handshake ends; a start sent too early is refused, so retry until it switches.
        for attempt in range(20):
            await asyncio.sleep(1.0)
            try:
                await mis.start_mission()
            except Exception as exc:
                log(f"start refused ({exc}); retrying")
                continue
            mode = await tel.flight_mode()
            if getattr(mode, "name", str(mode)) == "MISSION":
                break
        else:
            raise SystemExit("PX4 would not start the mission")
        t_start_us = clock.now_us()
        log(f"mission started (attempt {attempt + 1})")
        deadline = time.monotonic() + 2.5 * sum(laps) + 120
        while not await mis.is_mission_finished():
            if time.monotonic() > deadline:
                raise SystemExit("mission did not finish in time")
            await asyncio.sleep(0.5)
        log("mission finished")
        await asyncio.sleep(2.0)
        await act.land()
        await wait_for(tel.subscribe_in_air(), lambda x: not x, 90, "landing")
        await asyncio.sleep(3.0)
        pump_task.cancel()

    info = {"mode": "mission", "config": cfg, "params": params, "plan": plan.summary(), "t0_ap_us": None,
            "mission_start_ap_us": t_start_us, "mission_speed_m_s": speed, "waypoints": len(items),
            "bend_step_m": bend_step,
            "track": {"lane": track.lane, "R": track.R, "L": track.L, "length": track.length},
            "placement": {"north0": place.north0, "east0": place.east0, "bearing_deg": place.bearing_deg},
            "offset_m": offset, "altitude_m": alt}
    (out / "run.json").write_text(json.dumps(info, indent=1))
    return info


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", nargs="?")
    ap.add_argument("--out", default="runs/mission")
    ap.add_argument("--laps", type=int)
    ap.add_argument("--lap", type=float)
    ap.add_argument("--bend-step", type=float, default=7.2, help="waypoint spacing on the bends (m)")
    ap.add_argument("--ulog-dir")
    a = ap.parse_args(argv)
    cfg = cfgmod.load(a.config)
    if a.lap or a.laps:
        cfg["pace"]["laps_s"] = [float(a.lap or cfg["pace"]["laps_s"][0])] * (a.laps or len(cfg["pace"]["laps_s"]))
    t_start = time.time()
    out = Path(a.out)
    info = asyncio.run(run(cfg, out, a.bend_step))
    if a.ulog_dir:
        logs = [p for p in Path(a.ulog_dir).rglob("*.ulg") if p.stat().st_mtime >= t_start - 5]
        if logs:
            src = max(logs, key=lambda p: p.stat().st_mtime)
            shutil.copy2(src, out / "flight.ulg")
            info["ulog"] = str(out / "flight.ulg")
            (out / "run.json").write_text(json.dumps(info, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
