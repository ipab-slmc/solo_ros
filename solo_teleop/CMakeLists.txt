cmake_minimum_required(VERSION 3.0.2)
project(solo_teleop)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  geometry_msgs
  roscpp
  sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${dependencies}
)

add_executable(solo_teleop_joystick
  src/main_joystick.cpp
  src/solo_teleop_joystick.cpp
)
add_dependencies(solo_teleop_joystick ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(solo_teleop_joystick ${catkin_LIBRARIES})

add_executable(solo_teleop_keyboard
  src/main_keyboard.cpp
  src/solo_teleop_keyboard.cpp
)
add_dependencies(solo_teleop_keyboard ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(solo_teleop_keyboard ${catkin_LIBRARIES})

# Install
install(TARGETS solo_teleop_joystick solo_teleop_keyboard
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
