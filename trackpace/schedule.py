"""Pace schedule: where the virtual athlete is at every instant.

t = 0 is the moment the athlete crosses the start line (s = 0) at lap-1 pace.
Before that the drone runs up from a hover on the home straight so it is already at
speed on the line. Each lap k covers exactly one lap length in exactly laps[k] seconds,
including any speed change, so every line crossing is on time by construction:

    crossing k is at t = laps[0] + ... + laps[k-1]

A speed change between laps is a constant-acceleration ramp at the start of the lap;
the plateau speed of that lap is solved so the lap still takes its target time.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Seg:
    t0: float        # start time (s, schedule clock)
    s0: float        # start distance (m)
    v0: float        # start speed (m/s)
    a: float         # constant acceleration (m/s^2)
    dur: float       # duration (s)
    kind: str        # "runup", "ramp", "pace", "stop"
    lap: int         # 1-based lap index, 0 for run-up / stop

    @property
    def t1(self) -> float:
        return self.t0 + self.dur

    @property
    def s1(self) -> float:
        return self.s0 + self.v0 * self.dur + 0.5 * self.a * self.dur ** 2

    @property
    def v1(self) -> float:
        return self.v0 + self.a * self.dur


@dataclass
class State:
    t: float
    s: float
    v: float
    a: float
    lap: int
    kind: str


@dataclass
class PacePlan:
    laps: list[float]
    lap_length: float
    runup_accel: float = 0.5
    runup_max_m: float = 80.0
    ramp_accel: float = 0.4
    stop_decel: float = 1.0
    segs: list[Seg] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        if not self.laps:
            raise ValueError("need at least one lap")
        if any(T <= 0 for T in self.laps):
            raise ValueError("lap times must be positive")
        P = self.lap_length
        v = P / self.laps[0]
        a_run = max(self.runup_accel, v * v / (2 * self.runup_max_m))
        t_run = v / a_run
        d_run = v * v / (2 * a_run)
        self.runup_distance = d_run
        self.runup_time = t_run
        segs = [Seg(-t_run, -d_run, 0.0, a_run, t_run, "runup", 0)]
        t, s = 0.0, 0.0
        for k, T in enumerate(self.laps, start=1):
            vp = P / T
            if abs(vp - v) > 1e-9:
                tr = 0.0
                for _ in range(100):                      # fixed point: ramp time <-> plateau speed
                    tr = abs(vp - v) / self.ramp_accel
                    if tr >= T:
                        raise ValueError(f"lap {k}: speed change {v:.2f}->{vp:.2f} m/s does not fit "
                                         f"in {T:.1f} s at {self.ramp_accel} m/s^2")
                    vp_new = (P - tr * v / 2) / (T - tr / 2)
                    if abs(vp_new - vp) < 1e-12:
                        break
                    vp = vp_new
                tr = abs(vp - v) / self.ramp_accel
                a = math.copysign(self.ramp_accel, vp - v)
                segs.append(Seg(t, s, v, a, tr, "ramp", k))
                s += v * tr + 0.5 * a * tr * tr
                t += tr
                v = vp
                rest = T - tr
            else:
                rest = T
            segs.append(Seg(t, s, v, 0.0, rest, "pace", k))
            t += rest
            s = k * P                                     # exact by construction; avoid drift
        t_stop = v / self.stop_decel
        segs.append(Seg(t, s, v, -self.stop_decel, t_stop, "stop", 0))
        self.segs = segs
        self._t0s = [g.t0 for g in segs]

    # ------------------------------------------------------------------
    @property
    def start(self) -> float:
        return self.segs[0].t0

    @property
    def end(self) -> float:
        return self.segs[-1].t1

    @property
    def end_distance(self) -> float:
        return self.segs[-1].s1

    def crossings(self) -> list[float]:
        """Planned times of the start/finish line: 0, T1, T1+T2, ..."""
        out, t = [0.0], 0.0
        for T in self.laps:
            t += T
            out.append(t)
        return out

    @property
    def max_speed(self) -> float:
        return max(max(g.v0, g.v1) for g in self.segs)

    def at(self, t: float) -> State:
        if t <= self.start:
            g = self.segs[0]
            return State(t, g.s0, 0.0, 0.0, 0, "hold")
        if t >= self.end:
            g = self.segs[-1]
            return State(t, g.s1, 0.0, 0.0, 0, "done")
        i = max(0, bisect.bisect_right(self._t0s, t) - 1)
        g = self.segs[i]
        tau = t - g.t0
        return State(t, g.s0 + g.v0 * tau + 0.5 * g.a * tau * tau, g.v0 + g.a * tau, g.a, g.lap, g.kind)

    def summary(self) -> dict:
        return {
            "laps_s": self.laps,
            "lap_length_m": self.lap_length,
            "total_s": sum(self.laps),
            "runup_m": round(self.runup_distance, 2),
            "runup_s": round(self.runup_time, 2),
            "max_speed_m_s": round(self.max_speed, 3),
            "segments": [dict(kind=g.kind, lap=g.lap, t0=round(g.t0, 4), dur=round(g.dur, 4),
                              v0=round(g.v0, 4), v1=round(g.v1, 4), a=g.a) for g in self.segs],
        }
