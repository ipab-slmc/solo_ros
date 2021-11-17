cmake_minimum_required(VERSION 3.0.2)
project(solo_driver)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  sensor_msgs  # Temp
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})
find_package(Eigen3 REQUIRED)
find_package(master_board_sdk REQUIRED)
find_package(odri_control_interface REQUIRED)

# Build
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${dependencies}
  DEPENDS EIGEN3 master_board_sdk odri_control_interface)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${master_board_sdk_INCLUDE_DIRS}
  ${odri_control_interface_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/solo_driver.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  master_board_sdk::master_board_sdk
  odri_control_interface::odri_control_interface)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
