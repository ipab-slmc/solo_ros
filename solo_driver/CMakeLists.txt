cmake_minimum_required(VERSION 3.0.2)
project(solo_driver)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  # robot_interfaces
  sensor_msgs  # Temp
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${dependencies})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/solo_driver.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # TODO(JaehyunShim): ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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
