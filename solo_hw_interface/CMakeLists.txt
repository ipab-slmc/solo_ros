cmake_minimum_required(VERSION 3.0.2)
project(solo_hw_interface)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  controller_manager
  hardware_interface
  solo_driver
  urdf
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${dependencies}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(solo_control_node
  src/solo_control_node.cpp
  src/solo_hw_interface.cpp)
add_dependencies(solo_control_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(solo_control_node ${catkin_LIBRARIES} solo_driver)  # why solo_driver comes here?

# Install
install(TARGETS solo_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
