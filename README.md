# Robot Movement Package

## Overview
This Robot Movement package for ROS 2 facilitates autonomous navigation by dynamically adjusting its linear and angular velocities based on the robot's x-coordinate. This is achieved through real-time odometry data.

## Prerequisites
Ensure you have the following installed:
- ROS 2 (Tested on Foxy Fitzroy, but compatible with other distributions)
- `robot_urdf` package for simulation

### Installing `robot_urdf`
Navigate to your workspace:
```bash
cd ~/dev_ws
```
Clone the repository into your ROS 2 workspace, e.g., ~/dev_ws/src.
```bash
git clone https://github.com/CarmineD8/robot_urdf.git
```
Change branch to `ros2`:
```bash
git checkout ros2
```

### Robot Movement pkg Installation
Navigate to your workspace:
```bash
cd ~/dev_ws
```
Clone the repository into your ROS 2 workspace:
```bash
git clone https://github.com/grubino22/assignment2_rt.git
```
Change branch to 'ros2':
```bash
git checkout ros2
```
Build the package:
```bash
colcon build
```
After building, ensure your workspace is properly sourced by adding the following line to your .bashrc file:
```bash
source /root/dev_ws/install/local_setup.bash
```

## Usage
Firstly, run the simulation:
```bash
ros2 launch robot_urdf gazebo.launch.py 
```
Then, test the code running the robot movement node using the command:
```bash
ros2 run robot_movement move_robot
```
This command initiates the node that subscribes to odometry updates and controls the robot's movement based on its position.

## Node Details
### Subscriptions
`/odom` `(nav_msgs/msg/Odometry)`: Receives odometry data to determine the robot's position.
### Publications
`/cmd_vel` `(geometry_msgs/msg/Twist)`: Issues velocity commands based on the robot's current x-coordinate.
### How It Works
The node listens for updates on the `/odom` topic. Upon receiving odometry data, the odom_callback function triggers, updating the robot's position. Based on the position:
- Linear motion: Applies between x-coordinates 2.0 and 9.0.
- Right turn: Activates when x is greater than 9.0.
- Left turn: Activates when x is less than 2.0.
