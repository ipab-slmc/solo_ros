// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_teleop/solo_teleop_keyboard.hpp"

// Reference: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

int main(int argc, char * argv[])
{
  // Init
  ros::init(argc, argv, "solo_teleop_keyboard");
  solo_teleop::SoloTeleopKeyboard solo_teleop_keyboard();

  // Update
  ros::spin();

  //  Shutdown
  ros::shutdown();
  return 0;
}
