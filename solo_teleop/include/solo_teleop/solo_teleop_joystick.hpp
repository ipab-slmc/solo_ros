// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_TELEOP__SOLO_TELEOP_JOYSTICK_HPP_
#define SOLO_TELEOP__SOLO_TELEOP_JOYSTICK_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace solo_teleop
{
class SoloTeleopJoystick
{
public:
  SoloTeleopJoystick();
  ~SoloTeleopJoystick();

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

  // ROS Publisher
  ros::Publisher cmd_vel_pub_;

  // ROS Subscriber
  ros::Subscriber joy_sub_;
  void joy_callback(const sensor_msgs::Joy::ConstPtr & msg);

  void print_joyop();
  // TODO(JaehyunShim): print_vel should be curr vel? or ref vel?
  void send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z, double vel_ang_x, double vel_ang_y, double vel_ang_z);
  double enforce_vel_limit(double vel, double limit);
};
}  // namespace solo_teleop
#endif  // SOLO_TELEOP__SOLO_TELEOP_JOYSTICK_HPP_
