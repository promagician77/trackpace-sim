"""Score a flight from its PX4 ULog against the pace schedule.

    python -m trackpace.analyze runs/ff59            # finds the .ulg via run.json, or --ulog PATH

Position comes from vehicle_local_position_groundtruth (the simulator's truth, 50 Hz),
timestamps from the autopilot clock, the same clock the schedule was laid on.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog

from .schedule import PacePlan
from .track import Placement, Track

NAV_STATE = {3: "mission", 4: "loiter", 14: "offboard", 17: "takeoff", 18: "land"}


def _ds(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None


def project_all(track: Track, place: Placement, n: np.ndarray, e: np.ndarray):
    s = np.empty_like(n)
    d = np.empty_like(n)
    for i in range(len(n)):
        x, y = place.from_ned(float(n[i]), float(e[i]))
        s[i], d[i] = track.project(x, y)
    return s, d


def unwrap(s: np.ndarray, P: float) -> np.ndarray:
    """Continuous progress: the run-up (just before the line) is negative, lap k spans [(k-1)P, kP)."""
    out = s.astype(float).copy()
    k = -1 if s[0] > P / 2 else 0
    out[0] = s[0] + k * P
    for i in range(1, len(s)):
        ds = s[i] - s[i - 1]
        if ds < -P / 2:
            k += 1
        elif ds > P / 2:
            k -= 1
        out[i] = s[i] + k * P
    return out


def crossings(t: np.ndarray, sc: np.ndarray, P: float, n: int) -> list[float]:
    """Times where continuous progress passes 0, P, 2P ... (linear interpolation, 50 Hz)."""
    out = []
    i = 0
    for k in range(n + 1):
        target = k * P
        while i < len(sc) - 1 and not (sc[i] < target <= sc[i + 1]):
            i += 1
        if i >= len(sc) - 1:
            break
        f = (target - sc[i]) / (sc[i + 1] - sc[i])
        out.append(float(t[i] + f * (t[i + 1] - t[i])))
    return out


def stats(a: np.ndarray) -> dict:
    a = np.asarray(a, float)
    if a.size == 0:
        return {}
    return {"rms": float(np.sqrt(np.mean(a ** 2))), "mean": float(np.mean(a)),
            "max_abs": float(np.max(np.abs(a))), "p95_abs": float(np.percentile(np.abs(a), 95))}


def analyze(run_dir: Path, ulog_path: Path | None = None, decimate_hz: float = 10.0) -> dict:
    run = json.loads((run_dir / "run.json").read_text())
    ulog_path = Path(ulog_path) if ulog_path else run_dir / "flight.ulg"
    if not ulog_path.exists() and run.get("ulog"):
        ulog_path = Path(run["ulog"])
    tr = run["track"]
    track = Track(lane=tr["lane"], radius=tr["R"], straight=tr["L"])
    P = track.length
    pl = run["placement"]
    place = Placement(pl["north0"], pl["east0"], pl["bearing_deg"])
    offset, alt = run.get("offset_m", 0.0), run["altitude_m"]
    scheduled = run.get("t0_ap_us") is not None
    laps_s = run["plan"]["laps_s"]
    nlaps = len(laps_s)

    ul = ULog(str(ulog_path), ["vehicle_local_position_groundtruth", "vehicle_local_position",
                               "trajectory_setpoint", "vehicle_status"])
    gt = _ds(ul, "vehicle_local_position_groundtruth")
    if gt is None:
        raise SystemExit("ULog has no vehicle_local_position_groundtruth (simulation only)")
    t_us = gt["timestamp"].astype(np.int64)
    N, E, D = gt["x"], gt["y"], gt["z"]
    vN, vE = gt["vx"], gt["vy"]

    s_raw, d = project_all(track, place, N, E)

    # time base: schedule zero on the autopilot clock, or first line crossing for a mission
    t_ref_us = run["t0_ap_us"] if scheduled else None
    if not scheduled:
        # mission: start scoring at the first crossing after the vehicle is moving
        sc_tmp = unwrap(s_raw, P)
        tt = (t_us - t_us[0]) / 1e6
        c0 = crossings(tt, sc_tmp, P, 0)
        t_ref_us = t_us[0] + c0[0] * 1e6
    t = (t_us - t_ref_us) / 1e6

    # unwrap starting shortly before the run-up so take-off wander doesn't count laps
    pre = t >= (run["plan"]["segments"][0]["t0"] - 1.0 if scheduled else -30.0)
    i0 = int(np.argmax(pre))
    t, s_raw, d, N, E, D, vN, vE = (a[i0:] for a in (t, s_raw, d, N, E, D, vN, vE))
    sc = unwrap(s_raw, P)
    speed = np.hypot(vN, vE)

    plan = PacePlan(laps_s, P, run["config"]["pace"]["runup_accel"], run["config"]["pace"]["runup_max_m"],
                    run["config"]["pace"]["ramp_accel"], run["config"]["pace"]["stop_decel"])
    total = sum(laps_s)
    cross = crossings(t, sc, P, nlaps)
    planned = plan.crossings()
    lap_times = [cross[k + 1] - cross[k] for k in range(len(cross) - 1)]
    lap_err_ms = [1000 * (lap_times[k] - laps_s[k]) for k in range(len(lap_times))]

    win = (t >= 0) & (t <= total)
    if not scheduled:
        win = (t >= 0) & (t <= (cross[-1] if len(cross) > 1 else t[-1]))
    s_plan = np.array([plan.at(x).s for x in t])
    v_plan = np.array([plan.at(x).v for x in t])
    e_along = sc - s_plan
    xtrack = d - offset
    bend = np.array([track.is_bend(x % P) for x in sc])
    # distance from the nearest straight/bend junction along the lap
    joints = np.array([0.0, track.b1_end, track.back_end, track.b2_end, P])
    sm = np.mod(sc, P)
    to_joint = np.min(np.abs(sm[:, None] - joints[None, :]), axis=1)

    res: dict = {
        "mode": run["mode"], "ulog": ulog_path.name, "laps_target_s": laps_s,
        "lap_length_m": P, "lane": track.lane, "offset_m": offset, "altitude_m": alt,
        "params": run.get("params", {}),
        "crossings_s": cross, "lap_times_s": lap_times, "lap_error_ms": lap_err_ms,
        "lap_error_max_abs_ms": max((abs(x) for x in lap_err_ms), default=None),
        "crossing_error_ms": [1000 * (c - p) for c, p in zip(cross, planned)] if scheduled else None,
        "along_track_m": stats(e_along[win]) if scheduled else None,
        "cross_track_m": {"all": stats(xtrack[win]), "straights": stats(xtrack[win & ~bend]),
                          "bends": stats(xtrack[win & bend]),
                          "junction_5m": stats(xtrack[win & (to_joint < 5)]),
                          "bend_mean_inward_m": float(-np.mean(xtrack[win & bend]))},
        "altitude_m": stats((-D - alt)[win]),
        "speed_m_s": {"mean": float(np.mean(speed[win])), "std": float(np.std(speed[win])),
                      "straights_mean": float(np.mean(speed[win & ~bend])),
                      "bends_mean": float(np.mean(speed[win & bend])),
                      "min": float(np.min(speed[win])), "max": float(np.max(speed[win]))},
    }
    if scheduled:
        res["along_track_s"] = {k: (v / (P / laps_s[0]) if k != "mean" else v / (P / laps_s[0]))
                                for k, v in res["along_track_m"].items()}

    # per-lap and lap-to-lap repeatability (1 m bins of progress)
    per_lap = []
    bins = np.arange(0, P, 1.0)
    d_by_lap = []
    e_by_lap = []
    for k in range(min(nlaps, len(cross) - 1)):
        m = (sc >= k * P) & (sc < (k + 1) * P)
        if m.sum() < 10:
            continue
        sk = sc[m] - k * P
        order = np.argsort(sk)
        d_by_lap.append(np.interp(bins, sk[order], xtrack[m][order]))
        e_by_lap.append(np.interp(bins, sk[order], e_along[m][order]))
        per_lap.append({"lap": k + 1, "time_s": lap_times[k], "target_s": laps_s[k],
                        "cross_track_rms_m": float(np.sqrt(np.mean(xtrack[m] ** 2))),
                        "cross_track_max_m": float(np.max(np.abs(xtrack[m]))),
                        "along_track_rms_m": float(np.sqrt(np.mean(e_along[m] ** 2))) if scheduled else None,
                        "speed_mean": float(np.mean(speed[m])), "speed_std": float(np.std(speed[m]))})
    res["per_lap"] = per_lap
    if len(d_by_lap) >= 2:
        dl = np.array(d_by_lap)
        el = np.array(e_by_lap)
        res["repeatability"] = {
            "laps_compared": len(d_by_lap),
            "path_spread_max_m": float(np.max(np.ptp(dl, axis=0))),
            "path_std_mean_m": float(np.mean(np.std(dl, axis=0))),
            "timing_spread_max_m": float(np.max(np.ptp(el, axis=0))) if scheduled else None,
        }
        res["profile"] = {"s": bins[::2].tolist(),
                          "xtrack_mean": np.mean(dl, axis=0)[::2].round(4).tolist(),
                          "xtrack_by_lap": dl[:, ::2].round(4).tolist(),
                          "along_by_lap": el[:, ::2].round(4).tolist() if scheduled else None}

    # navigation error: what the EKF believed vs where the vehicle was
    lp = _ds(ul, "vehicle_local_position")
    if lp is not None:
        tl = (lp["timestamp"].astype(np.int64) - t_ref_us) / 1e6
        tw = (tl >= 0) & (tl <= (total if scheduled else (cross[-1] if len(cross) > 1 else tl[-1])))
        if tw.sum() > 10:
            tg = t
            gx, gy, gz = (np.interp(tl[tw], tg, a) for a in (N, E, D))
            gvx, gvy = np.interp(tl[tw], tg, vN), np.interp(tl[tw], tg, vE)
            sp = np.maximum(np.hypot(gvx, gvy), 0.1)
            ex, ey, ez = lp["x"][tw] - gx, lp["y"][tw] - gy, lp["z"][tw] - gz
            along_b = (ex * gvx + ey * gvy) / sp
            res["estimate_vs_truth"] = {
                "along_mean_m": float(np.mean(along_b)),
                "along_mean_ms": float(1000 * np.mean(along_b) / np.mean(sp)),
                "horizontal_rms_m": float(np.sqrt(np.mean(ex ** 2 + ey ** 2))),
                "height_rms_m": float(np.sqrt(np.mean(ez ** 2))),
            }

    # setpoint stream as PX4 received it
    ts = _ds(ul, "trajectory_setpoint")
    vs = _ds(ul, "vehicle_status")
    if ts is not None:
        tt = (ts["timestamp"].astype(np.int64) - t_ref_us) / 1e6
        m = (tt >= 0) & (tt <= total)
        if m.sum() > 2:
            # tracking vs the setpoint PX4 was given (logged at 5 Hz, interpolated to truth times)
            spn = np.interp(t[win], tt, ts["position[0]"])
            spe = np.interp(t[win], tt, ts["position[1]"])
            res["setpoint_tracking_m"] = stats(np.hypot(N[win] - spn, E[win] - spe))
    # setpoint stream as sent (setpoints.csv, autopilot-clock stamps); PX4's default log
    # samples trajectory_setpoint at 5 Hz, so it cannot show the stream itself
    csv_path = run_dir / "setpoints.csv"
    if csv_path.exists():
        arr = np.genfromtxt(csv_path, delimiter=",", names=True, usecols=(0, 1))
        m = (arr["t"] >= 0) & (arr["t"] <= total)
        dt = np.diff(arr["ap_us"][m]) / 1000.0
        if dt.size:
            res["setpoint_stream"] = {"count": int(m.sum()), "mean_interval_ms": float(np.mean(dt)),
                                      "p99_interval_ms": float(np.percentile(dt, 99)),
                                      "max_gap_ms": float(np.max(dt)),
                                      "failsafe_timeout_ms": 1000 * run.get("params", {}).get("COM_OF_LOSS_T", 1.0)}
    if vs is not None:
        modes = []
        last = None
        for ti, ns in zip(vs["timestamp"], vs["nav_state"]):
            if ns != last:
                modes.append({"t": round((int(ti) - t_ref_us) / 1e6, 2), "nav_state": NAV_STATE.get(int(ns), int(ns))})
                last = ns
        res["modes"] = modes

    # decimated series for the review site (track frame)
    step = max(1, int(round(50 / decimate_hz)))
    xs, ys = [], []
    for i in range(0, len(N), step):
        x, y = place.from_ned(float(N[i]), float(E[i]))
        xs.append(round(x, 3))
        ys.append(round(y, 3))
    ax, ay = [], []
    for i in range(0, len(N), step):
        st = plan.at(float(t[i])) if scheduled else None
        if st is None:
            ax.append(None); ay.append(None)
            continue
        px, py, _ = track.offset_point(st.s, 0.0)
        ax.append(round(px, 3)); ay.append(round(py, 3))
    res["series"] = {
        "t": t[::step].round(2).tolist(), "x": xs, "y": ys, "alt": (-D[::step]).round(3).tolist(),
        "s": sc[::step].round(2).tolist(), "xtrack": xtrack[::step].round(3).tolist(),
        "along": e_along[::step].round(3).tolist() if scheduled else None,
        "speed": speed[::step].round(3).tolist(), "v_plan": v_plan[::step].round(3).tolist() if scheduled else None,
        "athlete_x": ax, "athlete_y": ay,
    }
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--ulog")
    ap.add_argument("--out", help="write metrics JSON here (default <run_dir>/metrics.json)")
    a = ap.parse_args(argv)
    rd = Path(a.run_dir)
    res = analyze(rd, Path(a.ulog) if a.ulog else None)
    out = Path(a.out) if a.out else rd / "metrics.json"
    out.write_text(json.dumps(res))
    brief = {k: res[k] for k in ("mode", "lap_times_s", "lap_error_ms", "crossing_error_ms", "along_track_m",
                                 "cross_track_m", "altitude_m", "speed_m_s", "repeatability", "setpoint_stream",
                                 "setpoint_tracking_m") if k in res}
    print(json.dumps(brief, indent=1, default=lambda x: round(x, 4) if isinstance(x, float) else x))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
