# robotont\_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

[![CI](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 1. Installing and starting the driver

### 1. Install Ubuntu 22.04

### 2. Install ROS 2 Rolling and create a workspace [Creating a workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace)

### 3. Pull driver repo and install ROS dependencies

Navigate to workspace root folder
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

### 4. Build the driver

```bash
colcon build
```

### 5. Source install files and start the driver using the launch configuration

```bash
source install/local_setup.bash
ros2 launch robotont_driver driver_launch.py
```

Launch parameters are defined in 3 different places. With the following hierarchy (lower number overwrites lower number):
* 1: The parameters specified via command line, when using the launch file
* 2: The parameters defined in the launch file
* 3: The parameters hardcoded in the driver code

To change the launch parameters once, they can also be specified via the command line:

```bash
ros2 launch robotont_driver driver_launch.py device_name:='new_device_name_parameter'
```

## 2. Moving the robot

The drivers for the simulated and the real robot both subscribe to a `cmd_vel` topic. To move the robot, we have to publish velocity messages to this exact topic.

Here we use the teleop\_twist\_keyboard node, which translates command line keypresses to velocity messages and published these by default on `cmd_vel` topic.
Run the node
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* Follow the printout in the console to learn how to control the robot. Enjoy the ride!


## 3. Available plugins

### plugin\_odom

This plugin receives the ODOM packet from the robot and publishes the data on /odom (<nav_msgs::Odometry>) topic. This plugin also broadcasts an odom frame via TF.

### plugin\_motor

This plugin subscribes to cmd_vel topic and relays the velocities to robotont's motors
