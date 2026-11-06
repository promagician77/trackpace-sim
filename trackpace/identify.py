"""Identify the airframe's horizontal drag from any flight log.

    python -m trackpace.identify runs/calib/flight.ulg

Model (horizontal plane): a = f_h - c * v
  a   measured acceleration (differentiated velocity)
  f_h horizontal specific thrust = g * tilt vector (from attitude, with vertical balance)
  c   linear drag per unit mass (1/s), what the pacer adds to its feed-forward: a_ff += c * v

Least squares over the samples where the vehicle is moving. Works on any log with
attitude and velocity (a real flight's EKF estimate, or simulator truth).
"""
from __future__ import annotations

import argparse
import json

import numpy as np
from pyulog import ULog

G = 9.80665


def identify(ulog_path: str, min_speed: float = 2.0, truth: bool = True) -> dict:
    ul = ULog(ulog_path, ["vehicle_attitude", "vehicle_attitude_groundtruth",
                          "vehicle_local_position", "vehicle_local_position_groundtruth"])
    def ds(n):
        try:
            return ul.get_dataset(n).data
        except Exception:
            return None
    att = (ds("vehicle_attitude_groundtruth") if truth else None) or ds("vehicle_attitude")
    pos = (ds("vehicle_local_position_groundtruth") if truth else None) or ds("vehicle_local_position")
    tp = pos["timestamp"] / 1e6
    vx, vy, vz = pos["vx"], pos["vy"], pos["vz"]
    # smooth-differentiate velocity (central difference over 0.2 s)
    h = 5
    ax = (vx[2 * h:] - vx[:-2 * h]) / (tp[2 * h:] - tp[:-2 * h])
    ay = (vy[2 * h:] - vy[:-2 * h]) / (tp[2 * h:] - tp[:-2 * h])
    az = (vz[2 * h:] - vz[:-2 * h]) / (tp[2 * h:] - tp[:-2 * h])
    t = tp[h:-h]
    vxm, vym = vx[h:-h], vy[h:-h]
    ta = att["timestamp"] / 1e6
    q = [np.interp(t, ta, att[f"q[{i}]"]) for i in range(4)]
    w, x, y, z = q
    # body z axis (thrust acts along -z_body) in NED
    zb_n = 2 * (x * z + w * y)
    zb_e = 2 * (y * z - w * x)
    zb_d = 1 - 2 * (x * x + y * y)
    # thrust magnitude per unit mass from vertical balance: -T*zb_d + g = az  ->  T = (g - az)/zb_d
    T = (G - az) / zb_d
    fn, fe = -T * zb_n, -T * zb_e
    speed = np.hypot(vxm, vym)
    m = speed > min_speed
    # a - f = -c v   ->   c = -sum((a-f).v)/sum(v.v)
    rn, re = ax[m] - fn[m], ay[m] - fe[m]
    c = float(-(np.sum(rn * vxm[m]) + np.sum(re * vym[m])) / np.sum(vxm[m] ** 2 + vym[m] ** 2))
    resid = np.hypot(rn + c * vxm[m], re + c * vym[m])
    return {"drag_per_s": round(c, 4), "samples": int(m.sum()),
            "residual_rms_m_s2": round(float(np.sqrt(np.mean(resid ** 2))), 4),
            "speed_range_m_s": [round(float(speed[m].min()), 2), round(float(speed[m].max()), 2)],
            "source": "groundtruth" if truth and ds("vehicle_local_position_groundtruth") is not None else "estimate"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("--estimate", action="store_true", help="use EKF attitude/velocity instead of sim truth")
    a = ap.parse_args(argv)
    r = identify(a.ulog, truth=not a.estimate)
    print(json.dumps(r, indent=1))
    print(f"\nset in your config:\n[control]\ndrag_per_s = {r['drag_per_s']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
