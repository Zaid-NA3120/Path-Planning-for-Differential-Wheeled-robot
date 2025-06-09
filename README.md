# Path-Planning-for-Differential-Wheeled-robot


## Overview
This repository provides an implementation of two path planning algorithms — A* and Potential Field — integrated within ROS2 and the Gazebo simulator. Both algorithms have been specifically adapted to respect the nonholonomic constraints of a differential wheeled mobile robot (DMR). Traditional versions of these algorithms assume holonomic robots capable of moving in any direction, which is not applicable to DMRs.

To ensure the generated trajectories are feasible and executable by the robot, we implemented a path smoothing and control module using spline functions combined with kinematics-based feedback control.

The repository is designed with modularity in mind, making it easy to:

- Add new path planning algorithms.

- Integrate different control strategies.
## Project structure
- my_robot_description package: this package contains the URDF files that define the robot.
- my_robot_bringup package: this package contains the launch files used in this project. There are three launch files:
  
  1- display.launch.py/.xml: to display the robot using Rviz and check the tfs of the robot.
  
  2- my_robot_gazebo.launch.xml: to view the robot in the Gazebo simulator, where you can move it and manipulate the arm using plugins.
  
  3- robot_navigation.launch.py: this launch file is responsible for spawning the robot in gazbo simulator, read the map, plane the path based on the choosen algorithm, smooth the path and finally apply the control command to insure path tracking.
  
- my_robot_control package: this package contains the control algorithm (kinematics-based feedback) of the robot in addition to the path planning algorithm (potential field, A*).

## Installation constructions:
1. Create a workspace and inside the workspace directory Clone this repository.

```
mkdir ~/path_planning_proj
cd ~/path_planning_proj
git clone https://github.com/Zaid-NA3120/Path-Planning-for-Differential-Wheeled-robot.git
```
2. Build the workspace using Colcon build in your main directory.

```
cd ~/path_planning_proj
colcon build 
```
3. Source the workspace installation file.
```
source install/setup.bash
```
## Running the project
1. To display the robot on Rviz and check the tfs:
   ```
   ros2 launch my_robot_bringup display.launch.xml
   ```
2. To make the robot reach a desired goal:

```   
ros2 launch my_robot_bringup robot_navigation.launch.py goal_x:=4.0 goal_y:=2.0
```
## System Design and Algorithms
