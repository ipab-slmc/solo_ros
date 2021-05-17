cmake_minimum_required(VERSION 3.0.2)
project(solo_dashboard)

add_compile_options(-std=c++11)

# TODO(JaehyunShim): Check how catkin works and rewrite CMakeLists.txt file

# Find dependencies
set(dependencies
  geometry_msgs
  pluginlib
  qt_gui_cpp
  roscpp
  rqt_gui_cpp
  std_msgs
  pluginlib
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})
find_package(Qt5Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${dependencies}
)

# Build
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(src_files
  src/rosnode.cpp
  src/solo_dashboard.cpp
)

set(hd_files
  include/${PROJECT_NAME}/rosnode.hpp
  include/${PROJECT_NAME}/solo_dashboard.hpp
)

set(ui_files
  ui/solo_dashboard.ui
)

qt5_wrap_cpp(moc_files ${hd_files})
qt5_wrap_ui(ui_hd_files ${ui_files})

add_library(${PROJECT_NAME} SHARED
  ${src_files}
  ${moc_files}
  ${ui_hd_files}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ui
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
