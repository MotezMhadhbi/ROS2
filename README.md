# Dogbot Autonomous ROS Robot
this repository contains every thing you need for ros robot navigation outdoor and indoor 

This repository contains the code and configuration for Serbot, an autonomous mobile robot platform based on ROS2. Serbot integrates Arduino, Raspberry Pi, LIDAR (customizable), and an Ubuntu VM for advanced navigation, mapping, and visualization using RViz and SLAM.

Hardware Overview
Arduino: Handles low-level motor control and encoder feedback (tested with Arduino Uno).
Raspberry Pi: Runs ROS2 nodes for serial communication, sensor integration, robot control, and URDF robot description (tested with Raspberry Pi 5).
LIDAR: Provides 2D laser scans for mapping and navigation. Default setup uses YDLIDAR, but you can use any ROS2-compatible LIDAR (see below).
Ubuntu VM: Used for running ROS2 visualization tools (RViz), navigation stack, and SLAM algorithms.
Directory Structure
# Dogbot Basic/
├── For_Arduino/         # Arduino code for motor drivers and encoders
├── For_Raspberrypi/    # ROS2 nodes for serial, LIDAR, URDF (serbot_description), and robot control
├── For_Vm/             # ROS2 navigation, SLAM, and visualization packages
├── README.md           # Project documentation
serbot_description (in For_Raspberrypi): Contains the URDF robot model used for visualization and simulation in ROS2.
# System Architecture
Data Flow
Teleoperation/Navigation Commands (from Ubuntu VM or remote):
Sent as /cmd_vel ROS2 messages.
Raspberry Pi:
Receives /cmd_vel, sends velocity commands to Arduino via serial.
Publishes feedback from Arduino (encoder speeds) as /speed.
Runs LIDAR drivers; publishes /scan topic.
Publishes robot description (URDF) via serbot_description package.
# Arduino:
Receives velocity commands, drives motors, reads encoders.
Sends actual wheel speeds back to Raspberry Pi.
# Ubuntu VM:
Runs RViz for visualization, navigation, and SLAM (using LIDAR and odometry).
# ROS2 Nodes
serial_control: Bridges /cmd_vel (from navigation/teleop) to Arduino via serial, publishes /speed feedback.
serbot_controller: Computes odometry from wheel speeds, publishes /odom.
ydlidar_ros2_driver: Publishes LIDAR scans to /scan (see below for other LIDARs).
serbot_slam: Launches SLAM Toolbox for mapping.
serbot_navigation: Runs Nav2 stack for autonomous navigation.
serbot_description: Publishes the robot's URDF model.
Hardware Setup
Connect motor driver and encoders to Arduino as per motor_encoders_vel.ino pin definitions.
Connect Arduino to Raspberry Pi via USB.
Connect LIDAR to Raspberry Pi via USB/serial.
Ensure network connectivity between Raspberry Pi and Ubuntu VM (for ROS2 communication).
Software Setup
Prerequisites
ROS2 Humble or later (tested on ROS2 Jazzy)
Python 3.8 or later
Required ROS2 packages (Nav2, SLAM Toolbox, RViz, etc.)
Arduino IDE (for uploading firmware)
Installation
Create a ROS2 workspace (on both Raspberry Pi and Ubuntu VM):
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
Copy or clone the provided src folder contents into your workspace src directory.

# Install dependencies:

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
Build the workspace:
colcon build
Source the workspace:
source install/setup.bash
Usage Guide
# 1. Flash Arduino
Upload For_Arduino/motor_encoders_vel/motor_encoders_vel.ino to Arduino using Arduino IDE.
# 2. Launch Robot Software (on Raspberry Pi)
Start serial node (bridges /cmd_vel to Arduino, publishes /speed):

ros2 run serial_control serial_node

Start controller node (publishes /odom from Arduino feedback):

ros2 run serbot_controller serbot_controller

Launch LIDAR driver (default: YDLIDAR):

ros2 launch Lslidar_driver lslidar_driver.launch.py

frame_id: laser_frame
This is required for proper integration with the Serbot mapping and navigation stack. You can usually configure this in the LIDAR's launch or parameter YAML files.

Launch serbot_description to publish the URDF robot model:
ros2 launch serbot_description display.launch.py
# 3. Run SLAM or Navigation (on Ubuntu VM)
Launch SLAM:
ros2 launch serbot_slam serbot_slam.launch.py
Drive the robot to create a map.
Save the map (option 1, using launch file):
ros2 launch serbot_slam save_map.launch.py
Or save the map (option 2, using CLI):
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/maps/serbot_map
Launch Navigation:
ros2 launch serbot_navigation serbot_navigation.launch.py
(Optional) Launch static transform publisher:
ros2 launch serbot_navigation static_transforms.launch.py
Use RViz to set initial pose and navigation goals.
# Configuration Files
SLAM: For_Vm/src/serbot_slam/config/mapper_params_online_async.yaml
Navigation: For_Vm/src/serbot_navigation/config/controller_params_fast.yaml, advanced_amcl_params.yaml
Algorithms used
Global Planner: Smac Planner (Hybrid-A*)
Local Planner: Regulated Pure Pursuit Controller
Costmaps: Obstacle Layer + Inflation Layer, tuned for the robot/space
Behavior: customized Nav2 BTs for social navigation
Troubleshooting
Serial Issues: Check USB connections and device names.
Transform Errors: Ensure static transforms are running.
Navigation/SLAM Issues: Verify sensor data in RViz, adjust parameters as needed.
Failed to calculate odom: change base_frame in /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml from base_footprint to base_link
Tested Platforms
ROS2 Jazzy (recommended)
Raspberry Pi 5
Arduino Uno
Contributing
Fork the repository
Create a feature branch
Commit your changes
Push to the branch
Create a Pull Request
License
This project is licensed under the MIT License - see the LICENSE file for details.

Contact
For support or queries, contact:
zzmotez815@gmail.com
