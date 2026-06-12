# Industrial Robotics Course Project

This repository contains the project for the Industrial Robotics course. The system is built around the [Franka ROS 2 environment](https://github.com/frankarobotics/franka_ros2) and is focused on a pick-and-place task in simulation.

## Installation

### Using Docker

If you want to use Docker, copy the docker folder to your local environment, open a terminal inside it, and use the provided scripts:

```bash
./build.sh
```

Build the Docker image.

```bash
./run.sh
```

Start the container for the first time.

```bash
./exec.sh
```

Open additional terminals inside the running container.

The Docker image includes a fresh ROS 2 installation, so once inside the container follow the rest of the setup guidelines to install the required project packages and dependencies.

### Local installation

Update the package index:
```bash
sudo apt update
```

Install the ROS 2 development tools and the testing libraries used by the project:
```bash
sudo apt install ros-dev-tools libgtest-dev libgmock-dev
```

Clone the repository:
```bash
git clone https://github.com/BernardoBrogi/ROS2_project_franka.git
```

Enter in the ws:
```bash
cd ROS2_project_franka
```

Update rosdep:
```bash
rosdep update
```

Install the project dependencies:
```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

Build the workspace:
```bash
# Use --symlink-install to reduce disk usage and simplify development.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If the build appears to freeze, limit parallelism:
```bash
colcon build --parallel-workers 2
```

Source the workspace environment:
```bash
# Make the newly built ROS 2 packages available in the current shell.
source install/setup.bash
```


## Project Goal

The objective is to pick up a cube in simulation and place it autonomously inside a container using:

- Gazebo Ignition for the simulated environment
- MoveIt for motion planning and execution
- A simulated Intel RealSense RGB-D camera for perception

The robot must be able to detect the object, plan a grasp, pick the cube, and place it into the container without manual intervention.

## Simulation Setup

The simulated world includes:

- A Franka FR3 robotic arm
- A simulated Intel RealSense RGB-D camera

The camera streams both RGB data and point cloud information. These data streams can be visualized through ROS 2 topics and in RViz.

## Assignment Requirements

Students are expected to implement the following components:

1. A Gazebo world containing the cube, the container, and the obstacle.
2. A node that reads the camera data and estimates the pose of the cube.
3. A grasping and manipulation pipeline that uses the estimated pose to pick the cube.
4. Motion planning that avoids collisions with an obstacle placed between the robot and the cube.
5. A placement policy that autonomously places the cube inside the container.

## Obstacle Avoidance

Two possible approaches are:

- Use a collision object in the MoveIt planning scene, following the planning-around-objects tutorial.
- Use the MoveIt perception pipeline with OctoMap to build a 3D occupancy map from camera data and feed it to the planner for collision-aware trajectory generation.

Reference tutorial:

- https://moveit.picknik.ai/main/doc/tutorials/planning_around_objects/planning_around_objects.html
- https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html

## Data and Visualization

The camera provides:

- RGB images
- Point clouds

These outputs are intended to support object recognition, pose estimation, and debugging. RViz can be used to inspect the camera feeds, point cloud data, and the robot scene.

## Simulation Run Examples

Use this section to document example commands for starting the simulation, launching the robot stack, and opening RViz.

### Start the full simulation

```bash
ros2 launch franka_gazebo_bringup moveit_gazebo_franka_arm_example_controller.launch.py
```

## Environment Setup Examples

### Without Obstacle

Gazebo scene:

<img src="images/gazebo.png" alt="Gazebo environment without obstacle" width="52%"/>
RViz scene:

![RViz environment without obstacle](images/rviz.png)

### With Obstacle

Gazebo scene with obstacle:

![Gazebo environment with obstacle](images/gazebo_obstacle.png)

## Perception Topics

### Camera Topics

- Depth image topic: `/fr3/depth_camera/points`
- Point cloud topic: `/fr3/depth_camera/points`

### Suggested RViz Displays

- Camera image
- Point cloud
- Robot model
- TF frames



