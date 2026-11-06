"""Write a Gazebo (Harmonic) world with a standard 400 m track drawn on the ground.

    python gazebo/make_world.py > gazebo/worlds/track400.sdf

Visual only (no collisions), so it costs nothing in physics. The track sits where
config/gazebo_59.toml expects it: infield centre at local NED (20.0, -36.8), home straight
running north, so the drone spawned at the world origin stands on the lane-1 line of the
home straight, 20 m before its midpoint.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trackpace.track import KERB_RADIUS, LANE_WIDTH, STRAIGHT  # noqa: E402

CENTRE_N, CENTRE_E, BEARING = 20.0, -36.8, 0.0
LANES = 8
L = STRAIGHT
R_OUT = KERB_RADIUS + LANES * LANE_WIDTH
RED = "0.70 0.24 0.18 1"
GREEN = "0.27 0.52 0.25 1"
WHITE = "0.95 0.95 0.95 1"


def mat(rgba):
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def box(name, x, y, z, yaw, sx, sy, sz, rgba):
    return (f'<visual name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>'
            f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{mat(rgba)}"
            "<cast_shadows>false</cast_shadows></visual>")


def cyl(name, x, y, z, r, h, rgba):
    return (f'<visual name="{name}"><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>'
            f"<geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>{mat(rgba)}"
            "<cast_shadows>false</cast_shadows></visual>")


def track_visuals():
    v = []
    # red surface (stadium shape) and green infield, track frame: x along home straight, y left
    for i, cx in enumerate((L / 2, -L / 2)):
        v.append(cyl(f"red_bend{i}", cx, 0, 0.002, R_OUT, 0.004, RED))
        v.append(cyl(f"grass_bend{i}", cx, 0, 0.005, KERB_RADIUS, 0.004, GREEN))
    v.append(box("red_mid", 0, 0, 0.002, 0, L, 2 * R_OUT, 0.004, RED))
    v.append(box("grass_mid", 0, 0, 0.005, 0, L, 2 * KERB_RADIUS, 0.004, GREEN))
    # lane lines: kerb (k=0) to outer edge (k=LANES)
    seg_deg = 4.0
    n = int(180 / seg_deg)
    for k in range(LANES + 1):
        r = KERB_RADIUS + k * LANE_WIDTH
        w = 0.10 if k == 0 else 0.05
        for side, y in (("home", -r), ("back", r)):
            v.append(box(f"line{k}_{side}", 0, y, 0.009, 0, L, w, 0.004, WHITE))
        for b, (cx, th0) in enumerate(((L / 2, -90.0), (-L / 2, 90.0))):
            for j in range(n):
                th = math.radians(th0 + (j + 0.5) * seg_deg)
                chord = 2 * r * math.sin(math.radians(seg_deg / 2)) + 0.01
                v.append(box(f"line{k}_b{b}_{j}", cx + r * math.cos(th), r * math.sin(th), 0.009,
                             th + math.pi / 2, chord, w, 0.004, WHITE))
    # finish line (end of home straight) and 400 m lane-1 start = same line
    v.append(box("finish", L / 2, -(KERB_RADIUS + R_OUT) / 2, 0.010, 0, 0.05, R_OUT - KERB_RADIUS, 0.004, WHITE))
    return "\n".join(v)


def world():
    yaw = math.radians(90.0 - BEARING)   # track x axis -> compass bearing, Gazebo ENU
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="track400">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.62 0.75 0.88 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          {mat("0.42 0.55 0.36 1")}
        </visual>
      </link>
    </model>
    <model name="track400">
      <static>true</static>
      <pose>{CENTRE_E:.3f} {CENTRE_N:.3f} 0 0 0 {yaw:.6f}</pose>
      <link name="surface">
{track_visuals()}
      </link>
    </model>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.5 -0.8</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


if __name__ == "__main__":
    sys.stdout.write(world())
