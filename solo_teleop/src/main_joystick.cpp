// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_teleop/solo_teleop_joystick.hpp"

// Reference: https://github.com/ros-teleop/teleop_twist_joy

int main(int argc, char * argv[])
{
  // Init
  ros::init(argc, argv, "solo_teleop_joystick");
  solo_teleop::SoloTeleopJoystick solo_teleop_joystick();

  // Update
  ros::spin();

  //  Shutdown
  ros::shutdown();
  return 0;
}
