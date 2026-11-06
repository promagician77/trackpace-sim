# trackpace-sim: a PX4 drone pacing a 400 m track (SITL)

A simulated quadcopter flies a standard 400 m running-track oval at a programmed pace, under PX4 offboard
control, in Gazebo Harmonic (x500) or PX4's built-in SIH simulator. Every flight is then scored from its own
PX4 log (ULog): lap times, line crossings against the pace clock, gap to a virtual runner, distance off the
lane line, and lap-to-lap repeatability.

Python 3.10+, MAVSDK-Python 4.1, PX4 v1.17.0. No changes to PX4 itself.

## Measured results (runs/*/metrics.json)

| Run | Simulator | Laps | Worst lap vs target | Line crossings vs pace clock | Gap to runner (RMS) | Off lane-1 line (RMS) |
|---|---|---|---|---|---|---|
| 3 × 59.0 s | Gazebo Harmonic, x500 | 3 | ±21 ms | within ±42 ms | 0.17 m | 0.17 m |
| 5 × 59.0 s | PX4 SIH | 5 | ±61 ms | within ±59 ms | 0.44 m | 0.45 m |
| 72-66-62-59-56 s plan | PX4 SIH | 5 | ±80 ms | within ±85 ms | 0.44 m | 0.54 m |
| 59.0 s, flying 1.2 m outside lane 1 | PX4 SIH | 3 | ±100 ms | within ±120 ms | 0.37 m | 0.39 m |

Gazebo: lap-to-lap path spread 0.02 m (mean std), height 0.03 m RMS from the 3 m target.

Baselines on the same laps, same scoring (SIH unless marked):

| Method | Result |
|---|---|
| Position-only setpoints | laps within 62 ms of 59 s, but every crossing 1.05–1.13 s late (7.8 m behind the runner) |
| Pos + vel + acc, no drag term | 1.48 m wide on the bends, 0.93 m behind, worst lap ±173 ms |
| PX4 waypoint mission, 6.78 m/s | laps of 98.5 s and 91.4 s instead of 59 s; bends averaged 3.5 m/s |
| Gazebo, `EKF2_GPS_DELAY` left at 110 ms | laps within 7 ms, every crossing 83–92 ms late: the EKF placed the drone 0.69 m ahead of truth |

## Quick start

```bash
# 1. PX4 v1.17 SITL (Ubuntu 22.04/24.04 or WSL2). ubuntu.sh also installs Gazebo Harmonic.
git clone --recursive --branch v1.17.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && bash Tools/setup/ubuntu.sh && make px4_sitl_default

# 2. this project
cd trackpace-sim
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
pytest -q                                   # 51 tests

# 3a. Gazebo Harmonic + x500 on the 400 m track world (add --gui to watch)
scripts/sitl_gz.sh ~/PX4-Autopilot
python -m trackpace.fly config/gazebo_59.toml --out runs/gz59 \
    --ulog-dir ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log

# 3b. or PX4's built-in SIH quadcopter (no Gazebo)
scripts/sitl_sih.sh ~/PX4-Autopilot
python -m trackpace.fly config/pace_59.toml --out runs/ff59 \
    --ulog-dir ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log

# 4. score it
python -m trackpace.analyze runs/gz59
```

`bench/run_sih.sh` and `bench/run_gz.sh` reproduce every run behind the review site (each starts a fresh PX4).

## Configs

| File | What it flies |
|---|---|
| `config/pace_59.toml` | 5 laps at 59.0 s, lane 1, SIH (drag 0.99 identified) |
| `config/gazebo_59.toml` | 3 laps at 59.0 s on the drawn Gazebo track, x500 (drag 0.119, `EKF2_GPS_DELAY = 0`) |
| `config/pace_plan.toml` | a different target per lap: 72, 66, 62, 59, 56 s |
| `config/pace_lane2_offset.toml` | paces a lane-1 runner while flying 1.2 m outside them |
| `config/gps_example.toml` | real-track placement by GPS (infield centre + home-straight bearing) |

Every key is documented in `trackpace/config.py`. `[px4]` sets autopilot parameters before take-off.

## How it works

1. **Track** (`trackpace/track.py`): World Athletics geometry. Kerb radius 36.50 m, straights 84.39 m, lanes
   1.22 m. Lane 1 is measured 0.30 m outside the kerb (a 400.00 m lap), other lanes 0.20 m outside their inner
   line. The tests check the official 400 m staggers to 1 cm. The track is placed in PX4's local frame at the
   take-off point, at a local offset, or by GPS.
2. **Schedule** (`trackpace/schedule.py`): distance, speed and acceleration of a virtual runner over time. A
   run-up on the home straight brings the drone to pace at the start line. Pace changes between laps are
   ramps, and each lap's plateau speed is solved so the lap still takes exactly its target.
3. **Setpoints** (`trackpace/fly.py`): at 50 Hz, computed for the autopilot's clock (ODOMETRY timestamps,
   sliding-minimum offset), never from a queue:
   - position on the lane (with an optional sideways offset)
   - velocity along the tangent
   - acceleration = path acceleration + turn (v²/R) + identified drag (c·v)
   - nose on the tangent

   On the bends, a drone flying outside the runner's line flies faster by (R+d)/R so it stays level with the runner.
4. **Scoring** (`trackpace/analyze.py`): the simulator's ground truth from the ULog is projected onto the
   lane line. It reports line crossings (interpolated at 50 Hz), the gap to the runner, distance off the line
   on straights, bends and joins, height, lap-to-lap spread, estimator-vs-truth error, and the setpoint stream.
5. **Drag** (`trackpace/identify.py`): least squares on any flight log, `a = f_thrust − c·v`. SIH:
   1.008 from the EKF estimate (true 1.00). Gazebo x500: 0.119 from one calibration lap.

## Layout

```
trackpace/   track, schedule, config, fly (offboard pacer), mission (baseline), analyze, identify
gazebo/      make_world.py -> worlds/track400.sdf (8-lane track drawn to standard dimensions)
config/      example pace configs
scripts/     sitl_gz.sh (Gazebo Harmonic + x500), sitl_sih.sh (SIH)
bench/       the measured runs behind the site
runs/        per run: run.json (config, PX4 params, plan), metrics.json, analyze.txt, fly.log
site/        review site (static; Vercel: vercel.json, noindex)
print/       source of the 2-page PDF in dist/
tests/       51 tests
```

Flight logs (`flight.ulg`, 25–130 MB each) are not included. `runs/gz_cal/flight.ulg.xz` is the one
exception, so `trackpace.identify` can be tried on it. The bench scripts recreate the rest.

## Gazebo notes

- `scripts/sitl_gz.sh` starts `gz sim -s` with `gazebo/worlds/track400.sdf`. PX4 finds the running world
  and spawns the x500 at the origin, which sits on the lane-1 line of the home straight.
- Built and run here against Gazebo Harmonic 8.10 (gz-transport 13). With apt packages (`gz-harmonic`),
  `make px4_sitl_default` builds the bridge with no extra steps.

## Review site

`site/` is static and reads `site/data/*.json` (made by `python tools/export_site.py runs site/data`).
`vercel.json` publishes it with `X-Robots-Tag: noindex`. Deploy with `vercel deploy` from this folder, or
import the folder in the Vercel dashboard.

## Measured vs not measured

Measured: real-time SITL flights, PX4 v1.17.0, default PX4 parameters except where a config says otherwise.
Positions are simulator ground truth from the PX4 log, and times come from the autopilot clock.

Not measured:
- a real drone, wind, or people near it
- real GPS (non-RTK error is often 1–2 m, more than every error above)
- real sensor noise (the simulators' noise is lighter than hardware)

Each figure comes from a single run of 1–5 laps.
