cmake_minimum_required(VERSION 3.0.2)
project(solo_planner_py)

add_compile_options(-std=c++11)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS rospy)

# Build
catkin_package(
  CATKIN_DEPENDS rospy
)

# Install
catkin_install_python(PROGRAMS scripts/solo_planner_py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
