# SOLO ROS

## Install
- Ubuntu 20.04
- RT_PREEMPT patch
- ROS Noetic Ninjemys
- Dependencies in package.xml files using rosdep
- Dependencies in solo.rosinstall file using rosinstall

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
$ rosrun solo_planner solo_planner

# Rviz
$ roslaunch solo_description solo_rviz.launch

# Dashboard
$ rqt
(Plugin tab -> _Robot Dashboard_ -> Solo Dashboard)
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
