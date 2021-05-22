// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_TELEOP__SOLO_TELEOP_KEYBOARD_HPP_
#define SOLO_TELEOP__SOLO_TELEOP_KEYBOARD_HPP_

#include <termios.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace solo_teleop
{
class SoloTeleopKeyboard
{
public:
  SoloTeleopKeyboard();
  ~SoloTeleopKeyboard();

private:
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Linear Velocity
  double lin_vel_x_limit_;
  double lin_vel_x_step_size_;
  double lin_vel_y_limit_;
  double lin_vel_y_step_size_;
  double lin_vel_z_limit_;
  double lin_vel_z_step_size_;

  // Angular Velocity
  double ang_vel_x_limit_;
  double ang_vel_x_step_size_;
  double ang_vel_y_limit_;
  double ang_vel_y_step_size_;
  double ang_vel_z_limit_;
  double ang_vel_z_step_size_;

  // Terminal Settings
  struct termios orig_termios_;
  struct termios new_termios_;

  // ROS Publisher
  ros::Publisher cmd_vel_pub_;

  // Update
  void run();
  void print_keyop();
  // TODO(JaehyunShim): print_vel should be curr vel? or ref vel?
  void send_cmd_vel(
    double vel_lin_x, double vel_lin_y, double vel_lin_z,
    double vel_ang_x, double vel_ang_y, double vel_ang_z);
  double enforce_vel_limit(double vel, double limit);
};
}  // namespace solo_teleop
#endif  // SOLO_TELEOP__SOLO_TELEOP_KEYBOARD_HPP_
