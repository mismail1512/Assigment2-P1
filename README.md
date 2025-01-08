
---

# Goal Manager Program for Robot Navigation

This repository contains a **Goal Manager** node for robot navigation using ROS (Robot Operating System). The Goal Manager interacts with an action server to control a robot's movement based on user-specified targets. It allows you to set new targets, cancel current targets, and quit the program. The robot's state (position and velocity) is continuously monitored via odometry.

## Features
- **Set Target**: Send a new target position to the robot.
- **Cancel Target**: Cancel the currently active target.
- **Quit**: Exit the program.
- **Monitor Robot State**: Continuously publishes the robot’s state (position, velocity) during target execution.
- **Interactive User Interface**: Command-line interface for controlling the robot.
  
## Prerequisites
Ensure you have the following installed:
- **ROS Noetic** (or appropriate version) installed on your machine.
- **Python 3** and necessary libraries (rospy, actionlib, etc.)
- **Gazebo and Rviz** for simulation (optional for testing in a simulation environment).
  
### Dependencies
- `rospy`: for managing ROS nodes and communication.
- `actionlib`: for interacting with action servers.
- Custom messages:
  - `RobotState`: Contains robot status information (position, velocity).
  - `PlanningAction`: ROS Action for goal execution.

## Setup Instructions

1. **Clone the Repository**
   If you haven’t done so already, clone the repository into your ROS workspace:
   
   ```bash
   cd ~/catkin_ws/src
   git clone <your-repository-url>
   cd ..
   catkin_make
   source devel/setup.bash
   ```

2. **Launch the Simulation (Optional)**
   If you want to run the robot in simulation (Gazebo and Rviz), launch the appropriate simulation files. Make sure that your robot is already defined and the `robot_description` is loaded in your ROS parameter server.

   Example:
   ```bash
   roslaunch assignment_2_2024 sim_w1.launch
   ```

3. **Launch the Goal Manager Node**
   After setting up the environment, launch the `goal_manager` node. This will start the node that handles user interaction and robot navigation.

   ```bash
   roslaunch assignment_2_2024 assignment1.launch
   ```

4. **Run the Goal Manager**
   Once the node is running, you will see the interactive prompt in the terminal asking you to make choices (Set a new target, cancel, or quit).

## Using the Goal Manager

Once the node is running, the user will be presented with the following commands:

1. **Set a New Target**: 
   - The program will ask for an x-coordinate and a y-coordinate.
   - After entering the coordinates, the robot will begin navigating toward the target.
   - Example:
     ```plaintext
     Available Commands:
       [1] Set a new target
       [2] Cancel the active target
       [3] Quit
     Enter your choice: 1
     Enter x-coordinate: 2.0
     Enter y-coordinate: 3.0
     Target dispatched: x=2.0, y=3.0
     ```

2. **Cancel the Active Target**: 
   - If the robot is currently navigating to a target, the user can choose to cancel the target.
   - After cancellation, the program will ask if you want to set another target or exit the program.
   - Example:
     ```plaintext
     Available Commands:
       [2] Cancel the active target
       [3] Quit
     Enter your choice: 2
     Target operation canceled.
     Do you want to set another target? (yes/no): no
     Exiting the goal manager.
     ```

3. **Quit the Program**:
   - The user can quit the program at any time by choosing option `[3]`.
   - If there is an active goal, the program will stop it before exiting.
   - Example:
     ```plaintext
     Available Commands:
       [1] Set a new target
       [2] Cancel the active target
       [3] Quit
     Enter your choice: 3
     Exiting the goal manager.
     ```

### Detailed Steps for Interaction

1. **Set a New Target**:
   - The user selects option `[1]` to set a new target.
   - The program will prompt for the x and y coordinates.
   - Once the coordinates are entered, the robot will attempt to navigate to the specified target.

2. **Cancel an Active Target**:
   - The user selects option `[2]` to cancel the active goal.
   - The robot will stop its current navigation, and the goal is canceled.
   - After cancellation, the program will ask whether the user wants to set another goal or quit.

3. **Quit the Program**:
   - The user selects option `[3]` to quit the program at any time.

### Robot State Publishing

- The robot’s state (position and velocity) is published continuously as the robot moves toward the target.
- You can visualize the robot's state in **RViz** if you're using a simulation environment.
- If the robot has no active target, it will not publish the state updates.



## Example Use Case

1. **Set a New Target**:
   ```
   Available Commands:
     [1] Set a new target
     [2] Cancel the active target
     [3] Quit
   Enter your choice: 1
   Enter x-coordinate: 2.0
   Enter y-coordinate: 3.0
   Target dispatched: x=2.0, y=3.0
   ```

2. **Cancel the Active Target**:
   ```
   Available Commands:
     [2] Cancel the active target
     [3] Quit
   Enter your choice: 2
   Target operation canceled.
   Do you want to set another target? (yes/no): no
   Exiting the goal manager.
   ```

3. **Quit the Program**:
   ```
   Available Commands:
     [1] Set a new target
     [2] Cancel the active target
     [3] Quit
   Enter your choice: 3
   Exiting the goal manager.
   ```


---

## Conclusion

This Goal Manager node provides an interactive interface for controlling robot navigation by setting targets, canceling active goals, and quitting the program. It is intended to be flexible and easy to use in both real-world applications and simulations. By following the steps above, you can launch the program, interact with the robot, and test different use cases effectively.

Feel free to modify the code for additional functionality or improve the error handling as needed.

---
