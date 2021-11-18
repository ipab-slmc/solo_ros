# Solo ROS

The Solo ROS is a set of ROS interfaces for the [Solo12](https://open-dynamic-robot-initiative.github.io/) platform.

## Prerequisites
- Ubuntu 20.04
- ROS Noetic Ninjemys
- [rosdep](http://wiki.ros.org/rosdep)
- [vcstool](http://wiki.ros.org/vcstool)

## Installation
```sh
(Download solo_ros.repos to ~/catkin_ws)
$ cd ~/catkin_ws && wget -O solo_ros.repos 'https://raw.githubusercontent.com/ipab-slmc/solo_ros/devel/solo_ros.repos?token=AJGY62OKYWIZVMD7VSWYVPTBT5BYU'

(Download source dependency packages)
$ vcs import src < solo_ros.repos

(Download git submodules)
$ find . -maxdepth 3 -name .git -type d | rev | cut -c 6- | rev | xargs -I {} git -C {} submodule update --init

(Install binary dependency packages)
$ rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
```
- Others
  - pickle
  ```sh
  $ pip3 install pickle-mixin
  ```
  - crocoddyl (https://github.com/loco-3d/crocoddyl#installation-through-robotpkg)

## Run
```sh
# Physical Robot
$ roslaunch solo_hw_interface solo.launch

# Gazebo
(Standing on the ground)
$ roslaunch solo_gazebo solo.launch

(Floating in the air)
$ roslaunch solo_gazebo solo_fixed.launch

# Planner
# $ rosrun solo_planner solo_planner
$ roslaunch solo_planner_py solo_planner_py.launch

# Rviz
$ roslaunch solo_description solo_rviz.launch

# Dashboard
$ rqt
(Plugin tab -> _Robot Dashboard_ -> Solo Dashboard)

# Teleop
(Teleop keyboard)
$ rosrun solo_teleop solo_teleop_keyboard

(Teleop Joystick)
$ roslaunch solo_teleop solo_teleop_joystick.launch
```

# Reference
- Controller framework
  - http://wiki.ros.org/ros_control
  - https://github.com/ros-controls/ros_control
  - https://github.com/ros-controls/ros_controllers
  - http://wiki.ros.org/imu_sensor_controller
- Data communication
  - https://github.com/ipab-slmc/ipab_controller_msgs
- Motor & Sensor driver
  - https://github.com/open-dynamic-robot-initiative/solo
- Planner
  - https://github.com/paLeziart/quadruped-reactive-walking
- Robot description
  - https://github.com/open-dynamic-robot-initiative/robot_properties_solo
