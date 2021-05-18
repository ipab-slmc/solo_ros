// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <signal.h>

#include <controller_manager/controller_manager.h>

#include "solo_hw_interface/solo_hw_interface.hpp"

bool active = true;
void active_handler(int dummy)
{
  (void)(dummy);
  active = false;
}

int main(int argc, char * argv[])
{
  // Init
  ros::init(argc, argv, "solo_control_node");
  ros::NodeHandle root_nh("");
  ros::NodeHandle robot_hw_nh("~");

  // TODO(JaehyunShim): Rewrite hw_interface instantiation
  solo_hw_interface::SoloHwInterface hw_interface(root_nh, robot_hw_nh);
  controller_manager::ControllerManager cm(&hw_interface, root_nh);

  // Init HW interface
  if (!hw_interface.init(root_nh, robot_hw_nh)) {
    return -1;
  }

  // Update
  ros::Time curr_time;
  ros::Time last_time;
  ros::Duration elapsed;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // https://github.com/doosan-robotics/doosan-robot/blob/master/dsr_control/src/dsr_control_node.cpp#L73
  // https://github.com/ROBOTIS-SYS/aimbot_base/blob/d32fe11eac74f0f91f8ba814ab9983e253e4833c/aimbot_hw/src/control_node.cpp
  signal(SIGINT, active_handler);
  ros::Rate r(1000);  // 1000[Hz]
  last_time = ros::Time::now();
  while (ros::ok() && active) {
    curr_time = ros::Time::now();
    elapsed = curr_time - last_time;
    hw_interface.read();
    cm.update(ros::Time::now(), elapsed);
    hw_interface.write();
    r.sleep();  // sleep for 1[ms]
  }

  // Shutdown
  spinner.stop();
  ros::shutdown();
  return 0;
}
