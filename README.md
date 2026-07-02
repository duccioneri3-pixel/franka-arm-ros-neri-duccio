# Franka FR3 — Autonomous Cube Pick-and-Place (ROS 2)

Autonomous pick-and-place of a red cube with a Franka FR3 arm in Gazebo (Ignition),
using HSV + RGB-D perception and MoveIt 2 for collision-aware motion planning.

The robot detects a red cube, estimates its 3D pose from an RGB-D point cloud,
plans a grasp, lifts the cube while avoiding an obstacle, and places it on a target.

## What this repository contains

This repository contains **only the two packages I developed**:

- **`cube_detector`** (Python): detects the red cube via HSV color segmentation,
  estimates its 3D position from the RGB-D point cloud, transforms it into the
  `world` frame, and publishes it on `/cube_pose` (`geometry_msgs/PoseStamped`).
- **`cube_planner`** (C++): subscribes to `/cube_pose`, builds the MoveIt planning
  scene (obstacle, cube, and a ground/table collision plane), and executes the pick-and-place sequence
  (pre-grasp → approach → grasp → lift → transport → place → release → retreat)
  with `MoveGroupInterface`.

> **Note:** the Franka simulation environment (description, Gazebo bringup, MoveIt
> config, controllers) is **not** included here. It comes from the project base
> environment — see *Dependencies* below.

## Repository structure

```text
.
├── cube_detector/                  # Python package — red cube detection
│   ├── cube_detector/
│   │   ├── __init__.py
│   │   └── detector_node.py        # HSV detection + RGB-D pose estimation -> /cube_pose
│   ├── resource/
│   │   └── cube_detector           # ament package marker
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
│
└── cube_planner/                   # C++ package — MoveIt 2 pick-and-place
    ├── src/
    │   └── planner.cpp             # planning scene + grasp sequence + gripper control
    ├── launch/
    │   ├── full_demo.launch.py     # main launch: Gazebo + MoveIt + detector + planner
    │   └── planner.launch.py
    ├── worlds/
    │   └── my_world2.sdf           # Gazebo scene: cube, obstacle, target
    ├── CMakeLists.txt
    └── package.xml
```

## Dependencies

- **ROS 2 Humble**
- **MoveIt 2**
- **Gazebo (Ignition)**
- **Franka ROS 2 environment** — the base simulation this project builds on:
  [BernardoBrogi/ROS2_project_franka](https://github.com/BernardoBrogi/ROS2_project_franka)
  (Franka FR3 description, Gazebo world bringup, MoveIt config, RGB-D camera).
- Upstream Franka packages: [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2)

## Setup

This project runs **inside the Franka base workspace**. The two packages here are
added to that workspace's `src/`.

1. Set up the base workspace following its instructions:
   [BernardoBrogi/ROS2_project_franka](https://github.com/BernardoBrogi/ROS2_project_franka).

2. Clone these two packages into the workspace `src/`:

```bash
   cd <franka_workspace>/src
   git clone https://github.com/duccioneri3-pixel/franka-arm-ros-neri-duccio.git
```

3. Install ROS 2 dev tools and dependencies:

```bash
   sudo apt update
   sudo apt install ros-dev-tools libgtest-dev libgmock-dev
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

4. **Simulation world.** The pick-and-place scene (cube, obstacle, target) is
   defined in `cube_planner/worlds/my_world2.sdf` (included in this repo). Copy it
   into the base workspace at `franka_gazebo_bringup/worlds/`, and make sure the
   Gazebo bringup launch file loads it (i.e. it references `my_world2.sdf`).

5. Build and source:

```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
```
   If the build seems to freeze, limit parallelism:

```bash
   colcon build --parallel-workers 2
```

## Running the demo

```bash
ros2 launch cube_planner full_demo.launch.py
```

This brings up Gazebo + MoveIt, then starts `cube_detector` and `cube_planner`.
The robot detects the cube, plans, and executes the full pick-and-place.

Inspect the estimated cube pose:

```bash
ros2 topic echo /cube_pose
```

## Perception

The simulated RGB-D camera publishes:

- RGB image: `/fr3/depth_camera/image`
- Point cloud: `/fr3/depth_camera/points`

`cube_detector` segments the red cube in HSV, takes the bounding-box centroid,
samples a small window of the point cloud around it, and uses the median of the
valid 3D points (robust to NaNs and depth noise). The pose is transformed into the
`world` TF frame before publishing.

## Motion planning

`cube_planner` adds the obstacle, the cube, and a ground/table collision plane to the
MoveIt planning scene as collision objects. At grasp time the cube is **attached** to the gripper
(`fr3_hand`), so MoveIt accounts for its volume during lift and transport, and is
**detached** at release. The obstacle between the robot and the cube is avoided via
the MoveIt planning scene (planning around objects).

Reference tutorial:
https://moveit.picknik.ai/main/doc/tutorials/planning_around_objects/planning_around_objects.html

## Notes and known limitations

- **Gripper.** In this simulation only the `follow_joint_trajectory` action is
  exposed for the gripper, so grasping is done by closing the fingers to a
  position. The gripper command waits for the action result (not a fixed delay),
  so it is robust to slow simulation. On the **real** Franka, force-based grasping
  via the `franka_gripper` Grasp action would be used instead.
- **Cube holding in sim.** Because there is no force-based grasp in simulation,
  high friction on the cube is used to keep it held during transport.

## Demo

<!-- Drag-and-drop your simulation videos here when editing this README on GitHub.
     GitHub will host them and embed a playable link. Keep clips short (~10MB). -->

https://github.com/user-attachments/assets/efc5642c-2c9a-4f78-abaa-99196bcb7bdc





_Simulation videos to be added._

## License

MIT — see [LICENSE](LICENSE).
