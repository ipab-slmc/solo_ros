cmake_minimum_required(VERSION 3.0.2)
project(solo_gazebo)

add_compile_options(-std=c++11)

# Install
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
