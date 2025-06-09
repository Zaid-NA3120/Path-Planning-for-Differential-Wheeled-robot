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

### Path Planning Algorithms

#### A*

#### Potential Field

### Path Smoothing and Trajectory Generating

After generating a path using the planner (A* or Potential Field), the initial trajectory typically contains many intermediate points that are redundant for a differential mobile robot (DMR) to follow, especially in straight segments.

To optimize the path and reduce unnecessary points, we implemented a point filtering method based on slope comparison. This method preserves only the turning points (key waypoints) that represent a change in direction.
#### Algorithm description

For each consecutive triplet of points $p_{i-1}, p_i, p_{i+1}$ we compute the slopes of the two line segments:
- Slope between $p_{i-1}$ and $p_{i+1}$
- Slope between $p_{i-1}$ and $p_{i}$

If the difference between the two slopes is below a threshold, then the middle point is considered to be part of a straight line and can be discarded and the procedure is repeated with the point $p_{i+1}$ as the middle point. Otherwise, the middle point is kept and the window of calculation is moved by one point.

Finally, since most advanced control approaches require the full trajectory to ensure accurate tracking, we use spline functions to construct polynomials that represent the path as a function of time. This way, we can get polynomials that represent the velocity and acceleration of the robot.

### Controller Design

A differential-drive robot follows nonlinear kinematics, due to its dependence on orientation for motion. Controlling such a system directly is tricky because changes in the control input (linear and angular velocities) don't result in direct, linear changes in its pose.
The feedback linearization method tries to:

- Feedforward: Use the desired trajectory to compute nominal control inputs $(v_d, \omega_d)$

- Feedback: Apply corrections based on tracking error to steer the robot back to the trajectory.

The main idea is to express the error in a rotated frame aligned with the robot’s heading so that the control becomes simpler and more intuitive:

The reference trajectory:

$$
v_d = \sqrt{\dot{x}_d^2 + \dot{y}_d^2}
$$

$$
\theta_d = atan2(y_d, x_d) \\
$$

$$
\omega_d = \frac{\ddot{y}_d. \dot{x}_d - \ddot{x}_d. \dot{y}_d}{\dot{x}_d^2 + \dot{y}_d^2}
$$

Define the tracking error relative to the robot's frame:

$$
e_x = cos(\theta) (x_d - x) + sin(\theta) (y_d - y)
$$

$$
e_y = -sin(\theta) (x_d - x) + cos(\theta) (y_d - y)
$$

$$
e_{\theta} = \theta_d - \theta
$$

These equations represent the positional error projected into the robot’s coordinate frame, which makes the control independent of the global orientation. This is useful because it avoids having to control in a global coordinate system where orientation causes nonlinear effects.

Control law:

$$
v = v_d + k_1 e_x
$$

$$
\omega = \omega_d + k_2 e_y + k_3 sin(e_{\theta})
$$
