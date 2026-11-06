"""Collect measured runs into the review site's data files.

    python tools/export_site.py runs site/data

Writes summary.json (metrics of every run, no time series) and replay_<run>.json
(10 Hz series for the replay and charts). Everything comes from runs/*/metrics.json,
which trackpace.analyze computed from each flight's ULog.
"""
import json
import sys
from pathlib import Path

RUNS = [
    # id, label, sim, group
    ("gz59", "3 laps at 59.0 s", "Gazebo x500", "main"),
    ("ff59", "5 laps at 59.0 s", "PX4 SIH", "main"),
    ("plan", "Pace plan 72-66-62-59-56 s", "PX4 SIH", "main"),
    ("lane2", "59.0 s, flying 1.2 m outside lane 1", "PX4 SIH", "main"),
    ("nodrag59", "Feed-forward without drag term", "PX4 SIH", "baseline"),
    ("pos59", "Position-only setpoints", "PX4 SIH", "baseline"),
    ("mission59", "PX4 waypoint mission", "PX4 SIH", "baseline"),
    ("gz59_gpsdelay", "59.0 s with EKF2_GPS_DELAY left at 110 ms", "Gazebo x500", "baseline"),
    ("gz_cal", "Calibration lap (drag term off)", "Gazebo x500", "baseline"),
]

KEEP = ["mode", "laps_target_s", "lap_length_m", "lane", "offset_m", "altitude_m", "crossings_s",
        "lap_times_s", "lap_error_ms", "lap_error_max_abs_ms", "crossing_error_ms", "along_track_m",
        "along_track_s", "cross_track_m", "altitude_m", "speed_m_s", "per_lap", "repeatability",
        "setpoint_stream", "setpoint_tracking_m", "modes", "profile", "estimate_vs_truth"]


def main(runs_dir: str, out_dir: str) -> int:
    rd, od = Path(runs_dir), Path(out_dir)
    od.mkdir(parents=True, exist_ok=True)
    summary = []
    for rid, label, sim, group in RUNS:
        mp = rd / rid / "metrics.json"
        if not mp.exists():
            print("skip (not flown):", rid)
            continue
        m = json.loads(mp.read_text())
        run = json.loads((rd / rid / "run.json").read_text())
        item = {"id": rid, "label": label, "sim": sim, "group": group,
                "drag_per_s": run.get("drag_per_s"), "params": run.get("params", {}),
                "plan": run.get("plan", {}), "ulog": run.get("ulog_source") or m.get("ulog"),
                "late_ticks": run.get("late_ticks"), "ticks": run.get("ticks"),
                "mission_speed_m_s": run.get("mission_speed_m_s"), "waypoints": run.get("waypoints")}
        item.update({k: m.get(k) for k in KEEP})
        dt = rd / rid / "drag.txt"
        if dt.exists():
            item["identified_drag_per_s"] = float(dt.read_text())
        summary.append(item)
        (od / f"replay_{rid}.json").write_text(json.dumps(m["series"], separators=(",", ":")))
        print("exported", rid, len(m["series"]["t"]), "samples")
    (od / "summary.json").write_text(json.dumps(summary, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
