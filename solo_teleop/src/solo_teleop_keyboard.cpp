// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_teleop/solo_teleop_keyboard.hpp"

namespace solo_teleop
{
SoloTeleopKeyboard::SoloTeleopKeyboard()
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

  // Initialize ROS publisher
  // TODO(JaehyunShim): Need more consideration on queue size
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Change terminal mode
  tcgetattr(0, &orig_termios_);
  new_termios_ = orig_termios_;
  new_termios_.c_lflag &= ~ICANON;
  new_termios_.c_lflag &= ~ECHO;
  new_termios_.c_cc[VMIN] = 1;
  new_termios_.c_cc[VTIME] = 0;
  new_termios_.c_lflag &= ~ISIG;
  tcsetattr(0, TCSANOW, &new_termios_);

  // Print out keyboard operation
  print_keyop();

  run();
}

SoloTeleopKeyboard::~SoloTeleopKeyboard()
{
  // Reset terminal mode
  tcsetattr(0, TCSANOW, &orig_termios_);

  // Stop robot
  send_cmd_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void SoloTeleopKeyboard::run()
{
  uint8_t count = 0;
  double lin_vel_x_ref = 0.0;
  double lin_vel_y_ref = 0.0;
  double lin_vel_z_ref = 0.0;
  double ang_vel_x_ref = 0.0;
  double ang_vel_y_ref = 0.0;
  double ang_vel_z_ref = 0.0;

  try {
    while (true) {
      char ch = std::getchar();
      switch (ch) {
        case 'w':
          lin_vel_x_ref += lin_vel_x_step_size_;
          break;

        case 'x':
          lin_vel_x_ref -= lin_vel_x_step_size_;
          break;

        case 'd':
          lin_vel_y_ref += lin_vel_y_step_size_;
          break;

        case 'a':
          lin_vel_y_ref -= lin_vel_y_step_size_;
          break;

        case 'q':
          ang_vel_z_ref += ang_vel_z_step_size_;
          break;

        case 'e':
          ang_vel_z_ref -= ang_vel_z_step_size_;
          break;

        case 's':
          lin_vel_x_ref = 0.0;
          lin_vel_y_ref = 0.0;
          lin_vel_z_ref = 0.0;
          ang_vel_x_ref = 0.0;
          ang_vel_y_ref = 0.0;
          ang_vel_z_ref = 0.0;
          break;

        default:
          break;
      }

      send_cmd_vel(lin_vel_x_ref, lin_vel_y_ref, lin_vel_z_ref, ang_vel_x_ref, ang_vel_y_ref, ang_vel_z_ref);

      // Print keyboard operation every 10 commands
      count += 1;
      if (count == 10) {
        print_keyop();
        count = 0;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}

void SoloTeleopKeyboard::print_keyop()
{
  // TODO(JaehyunShim): Rewrite .. Better keyop?
  ROS_INFO("Key Operation\n");
  ROS_INFO("----------------------------------------\n");
  ROS_INFO("w/x: lin_vel_x +-\n");
  ROS_INFO("a/d: lin_vel_y +-\n");
  ROS_INFO("q/e: lin_vel_z +-\n");
  ROS_INFO("u/m: ang_vel_x +-\n");
  ROS_INFO("h/k: ang_vel_y +-\n");
  ROS_INFO("y/i: ang_vel_z +-\n");
  ROS_INFO("\n");
  ROS_INFO("lin_vel_x limit %.1lf\n", lin_vel_x_limit_);
  ROS_INFO("lin_vel_y limit %.1lf\n", lin_vel_y_limit_);
  ROS_INFO("lin_vel_z limit %.1lf\n", lin_vel_z_limit_);
  ROS_INFO("ang_vel_x limit %.1lf\n", lin_vel_x_limit_);
  ROS_INFO("ang_vel_y limit %.1lf\n", lin_vel_y_limit_);
  ROS_INFO("ang_vel_z limit %.1lf\n", lin_vel_z_limit_);
  ROS_INFO("\n");
  ROS_INFO("s: Stop\n");
}

// TODO(JaehyunShim): Add smoother

// TODO(JaehyunShim): why has to be reference, not pointer..?
void SoloTeleopKeyboard::send_cmd_vel(double vel_lin_x, double vel_lin_y, double vel_lin_z, double vel_ang_x, double vel_ang_y, double vel_ang_z)
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

// TODO(JaehyunShim): why has to be reference, not pointer..?
double SoloTeleopKeyboard::enforce_vel_limit(double vel, double limit)
{
  if (std::abs(vel) > limit) {
    vel = limit * (vel / std::abs(vel));
  }
  return vel;
}
}  // namespace solo_teleop
