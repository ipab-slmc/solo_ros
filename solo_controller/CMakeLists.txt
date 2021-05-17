cmake_minimum_required(VERSION 3.0.2)
project(solo_controller)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  controller_interface
  hardware_interface
  ipab_controller_msgs
  pluginlib
  realtime_tools
  sensor_msgs
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS ${dependencies}
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/solo_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
