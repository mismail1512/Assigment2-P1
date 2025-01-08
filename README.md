
# Robot Control using ROS 2 and Gazebo

## Overview

This project demonstrates how to simulate and control a robot in ROS 2 (Humble) using Gazebo for simulation and Python for controlling the robot's movement. The robot's velocity is controlled using a publisher, and its position is tracked via Odometry data. The robot is a simple differential drive robot with a wheel drive controller plugin in Gazebo.


## Project Structure

```bash
robot_urdf/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── robot4.xacro
├── launch/
│   └── gazebo.launch.py
├── config/
│   └── rviz.rviz
└── scripts/
    └── robot_mover.py
```

### Directories and Files:
- **`urdf/robot4.xacro`**: The robot's URDF model described using Xacro.
- **`launch/gazebo.launch.py`**: Launch file to start the Gazebo simulation with the robot model.
- **`config/rviz.rviz`**: Configuration file for RViz visualization.
- **`scripts/robot_mover.py`**: Python script that controls the robot's movement using the `/cmd_vel` topic and publishes its pose and velocity on `/robot_pose_velocity`.

## Setup Instructions



### 1. Source the Workspace

Source the workspace after building:

```bash
source ~/ws/install/setup.bash
```

### 2. Launch Gazebo

To launch the Gazebo simulation and load the robot model, run the following command:

```bash
ros2 launch robot_urdf gazebo.launch.py
```

This command will:
- Launch Gazebo with the specified robot model (loaded from `robot4.xacro`).
- Start the `robot_state_publisher` and `joint_state_publisher` nodes.
- Open RViz with the default configuration (loaded from `rviz.rviz`).

### 3. Move the Robot

In a new terminal, run the `robot_mover.py` node to move the robot by publishing velocity commands:

```bash
ros2 run robot_urdf robot_mover.py
```

This script will:
- Publish velocity commands (`linear.x` and `angular.z`) to the `/cmd_vel` topic.
- Track and log the robot's position using the `/odom` topic.
- Publish the robot's pose and velocity information to `/robot_pose_velocity`.

### 4. Echo the Robot's Pose and Velocity

To view the robot's pose and velocity, use the `ros2 topic echo` command:

```bash
ros2 topic echo /robot_pose_velocity
```

This will output the robot's pose and velocity data, showing the x, y position and linear velocity, which is being continuously updated as the robot moves.

## File Descriptions

### `robot_mover.py`

This Python script controls the robot's movement by publishing velocity commands to the `/cmd_vel` topic. The robot's position is also tracked using Odometry, and this information is logged to the terminal and published to the `/robot_pose_velocity` topic.

Key components:
- **Publisher**: Publishes to the `/cmd_vel` topic to control robot's velocity.
- **Subscriber**: Subscribes to the `/odom` topic to track the robot's position and velocity.
- **Timer**: Periodically sends velocity commands to make the robot move.
- **Logging**: Logs the robot's position and the velocity commands.

### `gazebo.launch.py`

The launch file that starts the simulation in Gazebo, including:
- Spawning the robot in the simulation environment.
- Starting the `robot_state_publisher` and `joint_state_publisher`.
- Launching RViz for visualization.
- Starting Gazebo with the specified world.



