# Target Package

## Overview
This package provides a comprehensive suite of tools to control and monitor a robot's movements in a ROS environment. It includes nodes for setting targets, retrieving the last set target, and a custom message and service for interfacing with the robot's positioning system.

## Package Added Contents

### Nodes
- **get_last_target.py**: Subscribes to goal messages and provides a service to retrieve the last received target.
- **set_target.py**: Allows users to set new targets interactively and cancels them if needed. It also publishes robot velocity and position updates.

### Custom Message
- **robot_par.msg**
```plaintext
float64 x
float64 y
float64 vel_x
loat64 vel_z
```
This message tracks the robot's position and velocity.

### Custom Service
- **Last_Target.srv**
```plaintext
---
geometry_msgs/Pose target_pose
```
This service returns the last known target pose of the robot.

### Launch File
- **complete_simulation.launch**: Launches the complete simulation environment including all necessary nodes and parameters for a predefined setup.

## Prerequisites
Ensure you have ROS installed and your environment is set up with all necessary dependencies for ROS and this package.

### Dependencies
- geometry_msgs
- nav_msgs
- actionlib
- rospy
## Installation
Navigate to your ROS workspace:
```bash
cd ~/dev_ws/src
```
Clone this repository
```bash
git clone https://github.com/grubino22/assignment2_rt.git
```
Change branch to 'ros1'
```bash
git checkout ros1
```
Go back to your worlspace and build the package:
```bash
cd ~/dev_ws/
catkin_make
```
After building, ensure your workspace is properly sourced by adding the following line to your `.bashrc` file:
```bash
source ~/dev_ws/devel/setup.bash
```
Finally, run
```bash
apt-get install xterm
```

## Usage
### Running the Nodes
To run the nodes individually, use the following commands:

#### Set Target Node
```bash
rosrun assignment_2_2024 set_target.py
```
#### Get Last Target Node
```bash
rosrun assignment_2_2024 get_last_target.py
```
### Launching the Complete Simulation
To start all nodes and the simulation environment, use the launch file:
```bash
roslaunch assignment_2_2024 complete_simulation.launch
```
## How It Works
### Set Target Node
Interactively allows the user to set or cancel navigation targets for the robot, responding to user input in the console. It handles user inputs to set new goals, cancel them, and provides feedback on the action's success or failure.

### Get Last Target Node
This node listens for target goals and, upon request via a ROS service, provides the last set target, allowing other components or users to know where the robot was last directed.

## Node Details
### Subscriptions
- set_target.py:
Subscribes to `/odom` for real-time position updates.
- get_last_target.py:
Subscribes to `/reaching_goal/goal` to track the latest target goals.
### Publications
- set_target.py:
Publishes robot's position and velocity on `/robot_position_velocity`.
### Services
- get_last_target.py:
Provides a service `/get_last_goal` that responds with the last known target pose.
