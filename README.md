# SOLO

# TODO
- [ ] Backporting Robot driver dependencies
- [ ] Backporting Sample Dashboard

## Install
- Ubuntu 20.04
- ROS Noetic Ninjemys
- Dependencies by rosdep
- Dependencies by rosinstall

## Run
```sh
# HW Control Node
$ roslaunch

# Gazebo
$ roslaunch solo_gazebo solo.launch

# Rviz
$ roslaunch solo_description solo_rviz.launch

# Dashboard
$ roscore
$ rqt
(Plugin tab -> _Robot Dashboard_ -> Solo Dashboard)
```

# Issue

# Reference
- https://github.com/open-dynamic-robot-initiative/robot_properties_solo
