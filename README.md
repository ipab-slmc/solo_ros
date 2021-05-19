# SOLO ROS

## Install
- Ubuntu 20.04
- ROS Noetic Ninjemys
- Dependencies by rosdep
- Dependencies in solo.rosinstall by rosinstall

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
- https://github.com/open-dynamic-robot-initiative/robot_properties_solo
