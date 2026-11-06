"""Load a pace config (TOML). See config/*.toml for annotated examples."""
from __future__ import annotations

import copy
import tomllib
from pathlib import Path

DEFAULTS = {
    "connection": {"url": "udpin://0.0.0.0:14540", "timeout_s": 30.0},
    "track": {
        "lane": 1,            # lane whose measurement line defines the pace
        "offset_m": 0.0,      # fly this far outside that line (+ outward); pace stays the runner's
        "altitude_m": 3.0,    # above the take-off point
        "bearing_deg": 0.0,   # compass bearing of the home straight, running direction
        "anchor": "takeoff",  # "takeoff": run-up starts where the drone took off
                              # "local": infield centre at centre_north/centre_east (m, local NED)
                              # "gps": infield centre at centre_lat/centre_lon
        "centre_lat": None,
        "centre_lon": None,
        "centre_north": None,
        "centre_east": None,
    },
    "pace": {
        "lap_time_s": 59.0,   # used when laps_s is not given
        "laps": 5,
        "laps_s": None,       # explicit per-lap targets, e.g. [70, 66, 62, 59]
        "runup_accel": 0.5,   # m/s^2 run-up on the home straight
        "runup_max_m": 80.0,  # longest run-up allowed (home straight is 84.39 m)
        "ramp_accel": 0.4,    # m/s^2 between laps of different pace
        "stop_decel": 1.0,
    },
    "control": {
        "mode": "ff",         # ff: position+velocity+acceleration, pv: position+velocity, pos: position only
        "rate_hz": 50.0,
        "hold_s": 4.0,        # hover at the run-up start, nose down the straight, before go
        "tail_s": 4.0,        # hover after stopping
        "transit_speed": 2.0, # m/s to the run-up start when anchor = "gps" or "local"
        "drag_per_s": 0.0,    # linear drag per unit mass, 1/s (python -m trackpace.identify flight.ulg)
    },
    "px4": {},                # parameters to set before flight, e.g. MPC_XY_VEL_MAX = 12.0
}

PARAMS_TO_RECORD = [
    "MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC", "MPC_Z_P",
    "MPC_XY_VEL_MAX", "MPC_XY_CRUISE", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_JERK_AUTO",
    "MPC_TILTMAX_AIR", "MPC_XY_TRAJ_P", "NAV_ACC_RAD", "COM_OF_LOSS_T",
]


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path: str | Path | None) -> dict:
    user = {}
    if path:
        with open(path, "rb") as f:
            user = tomllib.load(f)
    cfg = _merge(DEFAULTS, user)
    p = cfg["pace"]
    if not p.get("laps_s"):
        p["laps_s"] = [float(p["lap_time_s"])] * int(p["laps"])
    p["laps_s"] = [float(x) for x in p["laps_s"]]
    if cfg["control"]["mode"] not in ("ff", "pv", "pos"):
        raise ValueError("control.mode must be ff, pv or pos")
    if cfg["track"]["anchor"] not in ("takeoff", "gps", "local"):
        raise ValueError("track.anchor must be takeoff, local or gps")
    return cfg
