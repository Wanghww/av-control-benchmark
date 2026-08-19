<div align="center">

# AV Control Benchmark

**A hands-on study of autonomous vehicle control algorithms — from kinematic simulation to CARLA physics**

[中文文档](README_zh.md) · [Report an Issue](../../issues)

</div>

---

## Why this project

I wanted to *actually* understand how self-driving cars steer themselves — not just read about PID and MPC, but implement them, break them, and measure exactly how much better one algorithm is than another under real physics.

This repository is the result: five lateral control algorithms, implemented from scratch in C++, benchmarked across three levels of simulation fidelity, and finally driven inside CARLA. It's a learning and research project — a way to close the gap between control theory on paper and control theory that actually keeps a car on the road.

If you're studying autonomous driving control, or just curious how Apollo/Autoware-style control stacks are structured, I hope this is useful.

---

## Demo

![CARLA Demo](results/carla_demo_small.gif)

*Pure Pursuit controller driving a Tesla Model 3 in CARLA's Town03, closed-loop through ROS2 — reference trajectory generated on the fly, vehicle state fed back at 20Hz.*

---

## What's inside

Five controllers, implemented and compared under identical conditions:

| Controller | Type | Predicts future? | Handles constraints? |
|---|---|---|---|
| PID | Longitudinal (speed) | ❌ | ❌ |
| Pure Pursuit | Geometric path tracking | ❌ | ❌ |
| Stanley | Geometric (front-axle error) | ❌ | ❌ |
| LQR | Optimal state feedback | ❌ (infinite horizon) | ❌ |
| MPC | Receding-horizon optimization | ✅ (N-step) | ✅ (QP constraints) |

Tested across **three levels of simulation fidelity**, each one revealing a bigger gap between the controllers:

```
Layer 1 — Kinematic bicycle model (Python + C++)
   ↓  add tire slip + yaw dynamics
Layer 2 — Dynamic bicycle model (C++, RK4 integration)
   ↓  add real physics engine
Layer 3 — CARLA (PhysX, ROS2 closed loop)
```

**The core finding:** the kinematic model is *too forgiving* — it hides the real performance gap between simple geometric controllers and optimization-based ones. Only once tire dynamics enter the picture does the gap become obvious.

| Layer | Model | PP vs. best controller (double lane change) |
|---|---|---|
| 1. Kinematic | No tire slip, v < 8 m/s assumption | **6%** |
| 2. Dynamic | Linear tire model, slip angle β, yaw rate r | **39%** (15 m/s), 54% (18 m/s) |
| 3. CARLA | Real PhysX vehicle physics | Closed-loop demo validated |

![Dynamic model speed sweep](cpp_controllers/results/benchmark_dynamic_comparison.png)

---

## Results in detail

### Layer 1 — Kinematic model, ISO 3888 double lane change (8 m/s)

| Controller | RMSE | Compute time |
|---|---|---|
| Pure Pursuit | 0.067 m | 1.1 μs |
| Stanley | 1.30 m *(see note below)* | 1.1 μs |
| LQR | 0.062 m | 3.0 μs |
| MPC | 0.063 m | 74 μs |

> Stanley's poor result here isn't a bug — it's a known limitation of the control law at highway speed. Stanley's lateral-error correction term scales down with `1/v`, so at higher speeds it relies almost entirely on heading-error correction — which lags behind a fast-changing reference heading during a lane change. Stanley was designed (and shines) at low-speed maneuvers like parking.

![Speed sweep - kinematic](cpp_controllers/results/benchmark_speed_sweep.png)
![RMSE summary](cpp_controllers/results/benchmark_rmse_summary.png)
![Radar chart](cpp_controllers/results/benchmark_radar.png)

MPC's 74 μs compute time is **0.15% of a 20 Hz control cycle's 50 ms budget** — real-time constraints are a non-issue even at this scale.

![Timing comparison](cpp_controllers/results/benchmark_timing.png)

### Layer 2 — Dynamic bicycle model (tire slip + yaw inertia)

| Speed | Pure Pursuit | LQR (dynamic-adapted) | Improvement |
|---|---|---|---|
| 4 m/s | 0.087 m | 0.078 m | 10% |
| 8 m/s | 0.070 m | 0.069 m | 2% |
| 12 m/s | 0.077 m | 0.065 m | 16% |
| **15 m/s** | **0.142 m** | **0.087 m** | **39%** |
| 18 m/s | 0.238 m | 0.109 m | 54% |

Pure Pursuit's error grows sharply above 12 m/s as tire slip angle becomes significant — it only reasons about current geometry, not vehicle dynamics. LQR, adapted to use the dynamic model's yaw-rate response (`Cf·lf/Iz` instead of the kinematic `v/L`), stays stable.

### Layer 3 — CARLA

The ROS2 control loop (trajectory publisher → controller node → CARLA bridge) drives a real vehicle inside CARLA's physics engine. Key engineering challenges solved along the way:

- **Coordinate frames** — CARLA is left-handed (clockwise yaw), ROS is right-handed. Fixed via a local frame anchored to the vehicle's settled spawn pose.
- **Spawn settling** — sampling the reference pose immediately after `spawn_actor()` captures physics settling noise as heading error. Fixed by delaying 2 seconds.
- **Waypoint direction** — CARLA's `waypoint.next()` follows traffic direction, which can be opposite the vehicle's facing direction depending on spawn orientation. Detected and corrected automatically.

---

## Architecture

```
                     ┌─────────────────────┐
                     │  trajectory source   │  (fixed path / CARLA waypoints)
                     └──────────┬───────────┘
                                │ /trajectory
                     ┌──────────▼───────────┐
                     │   controller_node    │  PID + {PP | Stanley | LQR | MPC}
                     │        (C++)         │
                     └──────────┬───────────┘
                                │ /vehicle_cmd
              ┌─────────────────┴─────────────────┐
              │                                     │
   ┌──────────▼──────────┐              ┌──────────▼──────────┐
   │  sim_bridge_node.py  │              │ carla_bridge_node.py │
   │ (kinematic/dynamic)  │              │   (CARLA PhysX)      │
   └──────────┬──────────┘              └──────────┬──────────┘
              └─────────────────┬─────────────────┘
                                │ /vehicle_state (20 Hz)
                                ▼
                        back to controller_node
```

Swapping the physics backend — kinematic model, dynamic model, or CARLA — is a one-node change. The controller code never changes.

---

## Tech stack

| Component | Choice | Why |
|---|---|---|
| Controller core | C++17 | Real-time constraints, industry standard |
| QP solver (MPC) | OSQP (via osqp-eigen) | ADMM-based, sparse-matrix friendly, used by Apollo |
| Linear algebra | Eigen3 | Header-only, SIMD-optimized, expression templates |
| Middleware | ROS2 Humble | Decoupled node architecture, DDS-based |
| Simulator | CARLA 0.9.15 | Industry-standard AV simulator, real physics (PhysX) |
| Prototyping / viz | Python 3.10, NumPy, Matplotlib | Fast iteration before C++ porting |
| Testing | Google Test | Unit tests for controller edge cases |
| Build | CMake 3.22+ | ROS2 / C++ standard |

---

## Project structure

```
av-control-benchmark/
├── python_prototype/          # Layer 0: Python algorithm validation
│   ├── models/                #   kinematic + dynamic bicycle models
│   ├── controllers/           #   PID, Pure Pursuit (Python reference impl)
│   └── carla_bridge/          #   early CARLA integration experiments
│
├── cpp_controllers/           # Layers 1-2: C++ controller core
│   ├── include/                #   bicycle_model, dynamic_bicycle_model,
│   │                           #   pid, pure_pursuit, stanley, lqr, mpc
│   ├── src/                    #   implementations + main.cpp benchmark driver
│   ├── tests/                  #   Google Test unit tests
│   ├── scripts/                #   matplotlib benchmark visualization
│   └── results/                #   generated charts + CSV data
│
├── ros2_ws/                   # Layer 3: ROS2 + CARLA integration
│   └── src/
│       ├── av_control_msgs/    #   Trajectory / VehicleState / VehicleCmd
│       └── av_controller/      #   controller_node, carla_bridge_node
│
└── results/                   # Top-level demo assets (GIF)
```

---

## Building and running

### C++ controllers + benchmark

```bash
sudo apt install build-essential cmake libeigen3-dev libgtest-dev
# OSQP + osqp-eigen must be built from source: https://osqp.org

cd cpp_controllers
mkdir build && cd build
cmake .. && make -j4
./run_simulation          # runs the full benchmark suite, writes results/*.csv
ctest                     # runs unit tests

cd ../scripts
python3 plot_dynamic_comparison.py   # regenerate charts from CSV
```

### ROS2 + CARLA

Requires CARLA 0.9.15 running (locally or on a reachable host) and ROS2 Humble installed.

```bash
cd ros2_ws
colcon build --packages-select av_control_msgs av_controller
source install/setup.bash

# with CARLA (adjust carla_host if not on localhost)
ros2 launch av_controller carla_system.launch.py spawn_index:=1 speed:=4.0

# or with the lightweight Python simulator (no CARLA needed)
ros2 launch av_controller full_system.launch.py
```

---

## What I'd explore next

- Non-linear MPC for high-slip-angle regimes where the linear tire model breaks down
- State estimation (EKF) fusing GPS + IMU, instead of reading ground-truth simulator state
- A full double-lane-change run inside CARLA aligned to real road geometry (the current demo shows straight-line closed-loop control; the quantitative double-lane-change comparison lives in layers 1–2)

---

## Author

An undergraduate student learning autonomous vehicle control by building one from scratch. Feedback and issues welcome.

## Acknowledgments

Architecture inspired by [Apollo](https://github.com/ApolloAuto/apollo) and [Autoware](https://github.com/autowarefoundation/autoware.universe)'s control module design. Built with assistance from Claude AI for debugging and documentation.

## License

MIT
