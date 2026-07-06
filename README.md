# Franka FR3 — Autonomous Cube Pick-and-Place (ROS 2)

Autonomous pick-and-place of a red cube with a Franka FR3 arm in Gazebo
(Ignition), using HSV + RGB-D perception and MoveIt 2 for collision-aware
motion planning.

The robot detects a red cube, estimates its 3D pose from an RGB-D point cloud,
plans a grasp, lifts the cube while avoiding an obstacle, and places it on a
target — all from a single launch command, with no manual intervention.

The task is available in two flavours: a **linear planner** and a
**Behavior Tree** version that adds an explicit fail-safe recovery branch and
sensing-based grasp verification.

> 📄 A full technical report (design, evaluation, results) is available in
> [`report/main.pdf`](report/main.pdf).
---

## 🎥 Demo videos

**Full pick-and-place run:**

<!-- trascina qui il video della demo completa -->

**Recovery behavior (fail-safe):**

<!-- trascina qui il video del recovery, se ce l'hai -->

---

## What this repository contains

This repository contains **only the two packages I developed**:

- **`cube_detector`** (Python): detects the red cube via HSV color
  segmentation, estimates its 3D position from the RGB-D point cloud (spatial
  median over a window **and** a temporal median filter), transforms it into
  the `world` frame, and publishes it on `/cube_pose`
  (`geometry_msgs/PoseStamped`).
- **`cube_planner`** (C++): subscribes to `/cube_pose`, builds the MoveIt
  planning scene (obstacle, cube, and a ground/table collision plane, all
  parametrised), and executes the pick-and-place sequence with
  `MoveGroupInterface`. Two executables are provided: a linear planner
  (`planner.cpp`) and a Behavior Tree planner (`planner_bt.cpp`).

> **Note:** the Franka simulation environment (description, Gazebo bringup,
> MoveIt config, controllers) is **not** included here. It comes from the
> project base environment — see *Dependencies* below.

---

## Repository structure

```
cube_detector/                    # Python package — red cube detection
├── cube_detector/
│   ├── __init__.py
│   └── detector_node.py          # HSV detection + RGB-D pose + temporal filter -> /cube_pose
├── resource/
│   └── cube_detector             # ament package marker
├── package.xml
├── setup.py
└── setup.cfg

cube_planner/                     # C++ package — MoveIt 2 pick-and-place
├── src/
│   ├── planner.cpp               # linear pick-and-place sequence
│   └── planner_bt.cpp            # Behavior Tree version (recovery + grasp check)
├── launch/
│   ├── full_demo.launch.py       # launch: Gazebo + MoveIt + detector + linear planner
│   ├── bt_demo.launch.py         # launch: Gazebo + MoveIt + detector + BT planner
│   ├── planner.launch.py
│   └── moveit_gazebo_custom_tolerance.launch.py   # local copy of the sim launch (relaxed start tolerance)
├── config/
│   └── scene_real.yaml           # parametrised scene template for real-robot deployment
├── worlds/
│   └── my_world2.sdf             # Gazebo scene: cube, obstacle, target
├── CMakeLists.txt
└── package.xml

benchmark.sh                      # headless (xvfb) success-rate benchmark
benchmark_results/                # CSV results of benchmark runs
report/                           # LaTeX technical report + compiled PDF
```

---

## Key features

- **Behavior Tree architecture** (`planner_bt.cpp`): the pick-and-place is a
  `Sequence` wrapped in a `Fallback` with a recovery branch
  (`DetachCube → OpenGripper → GoHome`). If any step fails, the robot releases
  the cube and returns to a safe home pose instead of stopping mid-motion.
- **Sensing-based grasp verification**: after closing the gripper, a
  `CheckGrasp` node reads the finger joints from `/joint_states` and compares
  the gripper width against the cube size, so the robot does not lift an empty
  gripper.
- **Parametrised planning scene**: obstacle/table dimensions and positions and
  the cube size are ROS 2 parameters (defaults reproduce the simulation).
  They can be overridden — e.g. via `config/scene_real.yaml` — for a real
  laboratory setup without recompiling.
- **Cartesian vertical motions**: approach, lift, place and post-release lift
  are straight-line Cartesian moves (with an OMPL fallback), avoiding
  unnecessary detours; transport uses OMPL to route around the obstacle.
- **Temporal pose filtering**: a median filter over the last N cube poses
  smooths perception noise and rejects outliers.
- **Detector gating**: the detector is disabled after the grasp (via
  `/detector_enable`) to keep logs and the scene clean during transport.

---

## Build

From the workspace root (with the base Franka environment already present):

```bash
colcon build --packages-select cube_detector cube_planner
source install/setup.bash
```

## Run

**Behavior Tree version (recommended):**

```bash
ros2 launch cube_planner bt_demo.launch.py
```

**Linear version:**

```bash
ros2 launch cube_planner full_demo.launch.py
```

Each launch brings up Gazebo and MoveIt first, then the detector, then the
planner. The task runs autonomously to completion.

---

## Benchmark

`benchmark.sh` runs the task headless many times (inside a virtual framebuffer,
`xvfb`) and classifies each run as *direct success*, *recovered* (task failed
but the robot returned safely home), or *timeout/crash*. Results are written to
a CSV.

```bash
./benchmark.sh 35 180      # 35 runs, 180s timeout per run
```

Over 35 runs, the task completed successfully in about 77% of cases, and in
every failure the recovery branch returned the robot safely to the home pose.
See the report for the full evaluation.

---

## Dependencies

- ROS 2 Humble
- MoveIt 2
- Gazebo (Ignition)
- The Franka FR3 simulation base environment (`franka_description`,
  `franka_gazebo_bringup`, `franka_fr3_moveit_config`, controllers). These are
  **not** part of this repository; the two packages here depend on them but do
  not modify them.
- `xvfb` (only for the headless benchmark)

---

## License

See [LICENSE](LICENSE).
