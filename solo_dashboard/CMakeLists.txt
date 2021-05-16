cmake_minimum_required(VERSION 3.0.2)
project(solo_dashboard)

add_compile_options(-std=c++11)

# Find dependencies
set(dependencies
  geometry_msgs
  qt_gui_cpp
  roscpp
  rqt_gui_cpp
  std_msgs
  Qt5Widgets
)
find_package(catkin REQUIRED COMPONENTS ${dependencies})

# Build
include_directories(
  include
)

set(src_files
  src/solo_dashboard.cpp
  src/rosnode.cpp
)

set(hd_files
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
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/solo_dashboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ui
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Test
# TODO(JaehyunShim): Add test stuff if needed
