# robotont\_driver
ROS driver package for robotont.

This package handles low-level communication between ROS and robotont hardware.

[![CI](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/robotont/robotont_driver/actions/workflows/industrial_ci_action.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 1. Installing and starting the driver from ROS distro [NOT YET AVAILABLE]

### 1. Ensure Your System's Distribution is Supported. ROS 2 Jazzy supports Ubuntu 24.04

### 2. Install ROS 2 Jazzy [Installing ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### 3. Install robotont driver
```bash
 sudo apt install ros-jazzy-robotont-driver 
```

### 4. Source install files and start the driver using the launch configuration
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch robotont_driver driver_launch.py
```

Launch parameters are defined in 3 different places. With the following hierarchy (lower number overwrites higher number):
* 1: The parameters defined in the parameters file
* 2: The parameters specified via command line, when using the launch file
* 3: The parameters hardcoded in the driver code

To change the launch parameters once, they can also be specified via the command line:

```bash
ros2 launch robotont_driver driver_launch.py device_name:='/dev/ttyACM0'
```

## 2. Installing and starting the driver using source code

### 1. Ensure Your System's Distribution is Supported. ROS 2 Jazzy supports Ubuntu 24.04

### 2. Install ROS 2 Jazzy and create a workspace [Creating a workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#creating-a-workspace)

### 3. Pull driver repo and install ROS dependencies

Navigate to workspace root folder
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

The driver also relies on the following packages:
* 1: io-context
* 2: serial-driver
* 3: asio-cmake-module

```bash
sudo apt install ros-jazzy-io-context ros-jazzy-serial-driver ros-jazzy-asio-cmake-module
```

### 4. Build the driver

Ensure you are in your workspace root folder and enter the following commands to install them:
```bash
colcon build
```

### 5. Source install files and start the driver using the launch configuration

```bash
source install/local_setup.bash
ros2 launch robotont_driver driver_launch.py
```

Launch parameters are defined in 3 different places. With the following hierarchy (lower number overwrites higher number):
* 1: The parameters defined in the parameters file
* 2: The parameters specified via command line, when using the launch file
* 3: The parameters hardcoded in the driver code

To change the launch parameters once, they can also be specified via the command line:

```bash
ros2 launch robotont_driver driver_launch.py device_name:='/dev/ttyACM0'
```


## 3. Moving the robot

The drivers for the simulated and the real robot both subscribe to a `cmd_vel` topic. To move the robot, we have to publish velocity messages to this exact topic.

Here we use the teleop\_twist\_keyboard node, which translates command line keypresses to velocity messages and published these by default on `cmd_vel` topic.
Run the node
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

* Follow the printout in the console to learn how to control the robot. Enjoy the ride!


## 4. Available plugins

### plugin\_odom

This plugin receives the ODOM packet from the robot and publishes the data on /odom (<nav_msgs::Odometry>) topic. This plugin also broadcasts an odom frame via TF.

### plugin\_motor

This plugin subscribes to cmd_vel topic and relays the velocities to robotont's motors
