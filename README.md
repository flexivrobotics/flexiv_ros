# Flexiv ROS

[![Industrial CI](https://github.com/flexivrobotics/flexiv_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/flexivrobotics/flexiv_ros/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

RDK APIs are wrapped into ROS packages in `flexiv_ros`. Key functionalities like real-time joint torque and position control, `ros_control` and MoveIt! integrations are implemented.

> [!IMPORTANT]
> For new features and bug fixes, please migrate to ROS 2 and use the [Flexiv ROS 2 Humble](https://github.com/flexivrobotics/flexiv_ros2).

## References

[Flexiv RDK main webpage](https://www.flexiv.com/software/rdk) contains important information like RDK user manual and network setup.

## Compatibility

| **Supported OS** | **Supported ROS distribution**         |
| ---------------------- | -------------------------------------------- |
| Ubuntu 20.04           | [Noetic Ninjemys](https://wiki.ros.org/noetic/) |

## Building

```bash
# install additional ROS packages
$ sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

# source ros noetic
$ source /opt/ros/noetic/setup.bash

# create a catkin workspace
$ mkdir -p flexiv_ros_ws/src && cd flexiv_ros_ws

# clone the packages
$ git clone https://github.com/flexivrobotics/flexiv_ros.git src/flexiv_ros

# update the rdk submodule
$ cd src/flexiv_ros
$ git submodule update --init --recursive

# install dependencies
$ cd flexiv_ros_ws
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# source the workspace
$ source ~/flexiv_ros_ws/devel/setup.bash
```

## Usage

### Enable RDK on robot server

In order to use ROS driver to control the robot, you need to enable RDK on the robot server. See the [RDK Getting Started guide](https://www.flexiv.com/software/rdk/manual/set_up_the_robot.html) for details.

### Control the robot

Make sure that the ROS packages are built successfully and you have sourced the workspace. It is highly recommended to use a direct wired connection between the robot and the workstation PC for real-time access to the robot. The workstation PC must be in the same subnet with the robot in order to connect.

To start the robot driver run the following command in a terminal:

```bash
$ roslaunch flexiv_bringup rizon_control.launch robot_ip:=[robot_ip] local_ip:=[local_ip]
```

> [!NOTE]
> Rizon 4 is the default robot type in the above launch command. If you want to control the Rizon 4s or Rizon 10, add the launch argument e.g. `rizon_type:=rizon4s`.

In another terminal, start the following launch file to run the rqt:

```bash
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

You can use the rqt GUI to control the robot in joint space.

### Using MoveIt!

Launch the robot driver and the MoveIt! interface:

```bash
$ roslaunch flexiv_moveit_config moveit_control.launch robot_ip:=[robot_ip] local_ip:=[local_ip]
```

You should get a RViZ window with the robot in the planning scene. You can use the `MotionPlanning` plugin in RViZ to control the robot.

### Robot States

The following ROS topics are published by the robot driver:

- `/joint_states`: Joint states of the robot. [[`sensor_msgs/JointState.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)]
- `/external_wrench_in_tcp`: Estimated external wrench applied on TCP and expressed in TCP frame $^{TCP}F_{ext}$ in force $[N]$ and moment $[Nm]$. [[`geometry_msgs/Wrench.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html)]
- `/external_wrench_in_base`: Estimated external wrench applied on TCP and expressed in base frame $^{0}F_{ext}$ in force $[N]$ and moment $[Nm]$. [[`geometry_msgs/Wrench.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html)]
- `/wrench`: Force-torque (FT) sensor raw reading in flange frame $^{flange}F_{raw}$ in force $[N]$ and moment $[Nm]$. The value is 0 if no FT sensor is installed. [[`geometry_msgs/Wrench.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Wrench.html)]
- `/tcp_pose`: Measured TCP pose in base frame $^{0}T_{TCP}$ in position $[m]$ and quaternion. [[`geometry_msgs/Pose.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html)]
