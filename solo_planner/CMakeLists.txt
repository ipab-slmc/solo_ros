cmake_minimum_required(VERSION 3.0.2)
project(solo_planner)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  ipab_controller_msgs
  # quadruped-reactive-walking
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

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/solo_planner.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
