"""Standard 400 m track geometry (World Athletics / IAAF "400 m Standard Track").

Track frame: origin at the centre of the infield, x along the home straight in the
running direction, y to the runner's left (towards the infield on the home straight).
Running is counter-clockwise seen from above.

Distance s is measured on the lane's measurement line: 0.30 m outside the kerb for
lane 1, 0.20 m outside the inner lane line for lanes 2-8. s = 0 is the finish line
(end of the home straight), which is also the lane-1 start of a 400 m race; the
first bend follows.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

KERB_RADIUS = 36.50       # m, inner edge of lane 1 (kerb)
STRAIGHT = 84.39          # m, length of each straight
LANE_WIDTH = 1.22         # m, including the 50 mm line on the right
LANE1_MEASURE = 0.30      # m, lane 1 measured 0.30 m outside the kerb
LANEN_MEASURE = 0.20      # m, other lanes measured 0.20 m outside the inner line


def lane_radius(lane: int) -> float:
    """Radius of the measurement line of a lane on the bends."""
    if not 1 <= lane <= 9:
        raise ValueError("lane must be 1..9")
    if lane == 1:
        return KERB_RADIUS + LANE1_MEASURE
    return KERB_RADIUS + LANE_WIDTH * (lane - 1) + LANEN_MEASURE


@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    tx: float          # unit tangent
    ty: float
    kappa: float       # curvature of the measurement line (1/R on bends, 0 on straights)
    segment: str       # "bend1", "back", "bend2", "home"


class Track:
    """One lane of a standard 400 m track."""

    def __init__(self, lane: int = 1, radius: float | None = None, straight: float = STRAIGHT):
        self.lane = lane
        self.R = radius if radius is not None else lane_radius(lane)
        self.L = straight
        self.bend = math.pi * self.R
        self.length = 2 * self.bend + 2 * self.L
        # segment boundaries in s
        self.b1_end = self.bend
        self.back_end = self.bend + self.L
        self.b2_end = 2 * self.bend + self.L

    # ---- forward map -------------------------------------------------
    def sample(self, s: float) -> Sample:
        """Point, tangent and curvature at distance s (wrapped to one lap)."""
        R, L = self.R, self.L
        s = s % self.length
        if s < self.b1_end:
            th = -math.pi / 2 + s / R
            return Sample(L / 2 + R * math.cos(th), R * math.sin(th),
                          -math.sin(th), math.cos(th), 1.0 / R, "bend1")
        if s < self.back_end:
            u = s - self.b1_end
            return Sample(L / 2 - u, R, -1.0, 0.0, 0.0, "back")
        if s < self.b2_end:
            th = math.pi / 2 + (s - self.back_end) / R
            return Sample(-L / 2 + R * math.cos(th), R * math.sin(th),
                          -math.sin(th), math.cos(th), 1.0 / R, "bend2")
        u = s - self.b2_end
        return Sample(-L / 2 + u, -R, 1.0, 0.0, 0.0, "home")

    def offset_point(self, s: float, d: float) -> tuple[float, float, Sample]:
        """Point d metres outside the measurement line (d > 0 = outward, to the runner's right)."""
        p = self.sample(s)
        # right-hand normal of a counter-clockwise path points outward
        return p.x + d * p.ty, p.y - d * p.tx, p

    # ---- inverse map -------------------------------------------------
    def project(self, x: float, y: float) -> tuple[float, float]:
        """(s, d): progress on the measurement line beside the point, and outward offset.

        On the bends the projection is radial, so a point flying outside the lane is
        mapped to the runner beside it (same angle), not to the same distance flown.
        """
        R, L = self.R, self.L
        if x > L / 2:
            th = math.atan2(y, x - L / 2)                   # bend 1 spans -pi/2..pi/2
            return (R * (th + math.pi / 2)) % self.length, math.hypot(x - L / 2, y) - R
        if x < -L / 2:
            th = math.atan2(y, x + L / 2)
            if th < 0:
                th += 2 * math.pi                          # bend 2 spans pi/2..3pi/2
            return self.back_end + R * (th - math.pi / 2), math.hypot(x + L / 2, y) - R
        if y > 0:
            return self.b1_end + (L / 2 - x), y - R
        return self.b2_end + (x + L / 2), -y - R

    def segment_of(self, s: float) -> str:
        return self.sample(s).segment

    def is_bend(self, s: float) -> bool:
        return self.sample(s).kappa > 0


@dataclass(frozen=True)
class Placement:
    """Where the track frame sits in the autopilot's local NED frame."""
    north0: float = 0.0          # NED position of the track-frame origin (infield centre)
    east0: float = 0.0
    bearing_deg: float = 0.0     # compass bearing of the home straight, running direction

    def to_ned(self, x: float, y: float) -> tuple[float, float]:
        b = math.radians(self.bearing_deg)
        # x axis = (cos b, sin b) in (N, E); y axis (runner's left) = (sin b, -cos b)
        return (self.north0 + x * math.cos(b) + y * math.sin(b),
                self.east0 + x * math.sin(b) - y * math.cos(b))

    def vec_to_ned(self, vx: float, vy: float) -> tuple[float, float]:
        b = math.radians(self.bearing_deg)
        return vx * math.cos(b) + vy * math.sin(b), vx * math.sin(b) - vy * math.cos(b)

    def from_ned(self, n: float, e: float) -> tuple[float, float]:
        b = math.radians(self.bearing_deg)
        dn, de = n - self.north0, e - self.east0
        return dn * math.cos(b) + de * math.sin(b), dn * math.sin(b) - de * math.cos(b)

    def yaw_deg(self, tx: float, ty: float) -> float:
        """NED yaw (0 = north, clockwise) of a track-frame direction."""
        vn, ve = self.vec_to_ned(tx, ty)
        return math.degrees(math.atan2(ve, vn))

    @staticmethod
    def anchored_at(track: Track, s: float, d: float, north: float, east: float,
                    bearing_deg: float) -> "Placement":
        """Placement that puts track point (s, d) at the given NED position."""
        x, y, _ = track.offset_point(s, d)
        p0 = Placement(0.0, 0.0, bearing_deg)
        n1, e1 = p0.to_ned(x, y)
        return Placement(north - n1, east - e1, bearing_deg)

    @staticmethod
    def from_gps(center_lat: float, center_lon: float, origin_lat: float, origin_lon: float,
                 bearing_deg: float) -> "Placement":
        """Infield centre given in GPS, local NED origin = autopilot's EKF global origin.

        Local tangent-plane approximation; error < 1 mm over a 200 m track.
        """
        r_earth = 6371000.0
        n = math.radians(center_lat - origin_lat) * r_earth
        e = math.radians(center_lon - origin_lon) * r_earth * math.cos(math.radians(origin_lat))
        return Placement(n, e, bearing_deg)
