// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_teleop/solo_teleop_joystick.hpp"

namespace solo_teleop
{
SoloTeleopJoystick::SoloTeleopJoystick()
: nh_(""),
  private_nh_("~")
{
  // Init ROS parameter
  // TODO(JaehyunShim): Reset default values.
  // TODO(JaehyunShim): Simplify using struct.
  private_nh_.param<double>("lin_vel_x_limit", lin_vel_x_limit_, 0.5);
  private_nh_.param<double>("lin_vel_x_step_size", lin_vel_x_step_size_, 0.01);

  private_nh_.param<double>("lin_vel_y_limit", lin_vel_y_limit_, 0.5);
  private_nh_.param<double>("lin_vel_y_step_size", lin_vel_y_step_size_, 0.01);

  private_nh_.param<double>("lin_vel_z_limit", lin_vel_z_limit_, 0.5);
  private_nh_.param<double>("lin_vel_z_step_size", lin_vel_z_step_size_, 0.01);

  private_nh_.param<double>("ang_vel_x_limit", ang_vel_x_limit_, 5.0);
  private_nh_.param<double>("ang_vel_x_step_size", ang_vel_x_step_size_, 0.10);

  private_nh_.param<double>("ang_vel_y_limit", ang_vel_y_limit_, 5.0);
  private_nh_.param<double>("ang_vel_y_step_size", ang_vel_y_step_size_, 0.10);

  private_nh_.param<double>("ang_vel_z_limit", ang_vel_z_limit_, 5.0);
  private_nh_.param<double>("ang_vel_z_step_size", ang_vel_z_step_size_, 0.10);

  // Init ROS publisher and subscriber
  // TODO(JaehyunShim): Need more consideration on queue size
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub_ = nh_.subscribe("joy", 10, &SoloTeleopJoystick::joy_callback, this);

  // Print out joystick operation
  print_joyop();
}

SoloTeleopJoystick::~SoloTeleopJoystick()
{
  // Stop robot
  send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Reference: http://wiki.ros.org/joy
void SoloTeleopJoystick::joy_callback(const sensor_msgs::Joy::ConstPtr & msg)
{
  static uint8_t count = 0;
  static double lin_vel_ref = 0.0;
  static double ang_vel_z_ref = 0.0;

  // TODO(JaehyunShim): Key recheck needed

  // Boost x, y direction speed
  if (msg->axes.at(3) <= -0.9) {
  }

  // Boost yaw direction speed
  if (msg->axes.at(3) <= -0.9) {
  }

  // Move in x direction
  if (msg->axes.at(1) >= 0.9) {
    send_cmd_vel(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  } else if (msg->axes.at(1) <= -0.9) {
    send_cmd_vel(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // Move in y direction
  else if (msg->axes.at(0) >=  0.9) {
    send_cmd_vel(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  } else if (msg->axes.at(0) <= -0.9) {
    send_cmd_vel(0.0, -1.0, 0.0, 0.0, 0.0, 0.0);
  }

  // Move in yaw direction
  else if (msg->axes.at(6) == -1.0) {
    send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  } else if (msg->axes.at(6) == -1.0) {
    send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, -1.0);
  }

  // Stop
  else if (msg->buttons.at(0) == 1) {
    send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // Print keyboard operation every 10 commands
  count += 1;
  if (count == 10) {
    print_joyop();
    count = 0;
  }
}

void SoloTeleopJoystick::print_joyop()
{
  // TODO(JaehyunShim): Rewrite .. Better keyop?
  ROS_INFO("Joy Operation\n");
  ROS_INFO("----------------------------------------\n");
  ROS_INFO("Left Stick u/d: Forward/Backward Velocity\n");
  ROS_INFO("Left Stick l/r: Left/Right Velocity\n");
  ROS_INFO("Right Stick l/r: Angular Velocity\n");
  ROS_INFO("L2 Button: Stop\n\n");

  ROS_INFO("Forward/Backward Velocity Limit %.1lf\n", lin_vel_x_limit_);
  ROS_INFO("Left/Right Velocity Limit %.1lf\n", lin_vel_y_limit_);
  ROS_INFO("Angular Velocity Limit %.1lf\n", ang_vel_z_limit_);
}

// TODO(JaehyunShim): Add smoother

// TODO(JaehyunShim): why has to be reference, not pointer..?
void SoloTeleopJoystick::send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z, double vel_ang_x, double vel_ang_y, double vel_ang_z)
{
  // Enforce velocity limit
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = enforce_vel_limit(vel_lin_x, lin_vel_x_limit_);
  cmd_vel_msg.linear.y = enforce_vel_limit(vel_lin_y, lin_vel_y_limit_);
  cmd_vel_msg.linear.z = enforce_vel_limit(vel_lin_z, lin_vel_z_limit_);
  cmd_vel_msg.angular.x = enforce_vel_limit(vel_ang_x, ang_vel_x_limit_);
  cmd_vel_msg.angular.y = enforce_vel_limit(vel_ang_y, ang_vel_y_limit_);
  cmd_vel_msg.angular.z = enforce_vel_limit(vel_ang_z, ang_vel_z_limit_);
  cmd_vel_pub_.publish(cmd_vel_msg);

  ROS_INFO("vel_lin_x:: %.2lf\n", vel_lin_x);
  ROS_INFO("vel_lin_y: %.2lf\n", vel_lin_y);
  ROS_INFO("vel_lin_z: %.2lf\n", vel_lin_z);
  ROS_INFO("vel_ang_x:: %.2lf\n", vel_ang_x);
  ROS_INFO("vel_ang_y: %.2lf\n", vel_ang_y);
  ROS_INFO("vel_ang_z: %.2lf\n", vel_ang_z);
}

double SoloTeleopJoystick::enforce_vel_limit(double vel, double limit)
{
  if (std::abs(vel) > limit) {
    vel = limit * (vel / std::abs(vel));
  }
  return vel;
}
}  // namespace solo_teleop
