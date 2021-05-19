// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <solo_driver/solo_driver.hpp>

#include <memory>
#include <vector>

namespace solo_driver
{
void SoloDriver::init(uint8_t joint_size)
{
  joint_size_ = joint_size;
  pos_.resize(joint_size_);
  vel_.resize(joint_size_);
}

std::vector<double> SoloDriver::read_joint_pos()
{
  // Dummy
  for (size_t i = 0; i < joint_size_; i++) {
    pos_.at(i) = 0.1;
    // ROS_INFO("Joint %d Position: %lf", i, pos_.at(i));
  }

  return pos_;
}

std::vector<double> SoloDriver::read_joint_vel()
{
  // Dummy
  for (size_t i = 0; i < joint_size_; i++) {
    vel_.at(i) = 0.2;
    // ROS_INFO("Joint %d Velocity: %lf", i, vel_.at(i));
  }

  return vel_;
}

sensor_msgs::Imu SoloDriver::read_imu()
{
  // Dummy
  imu_.orientation.x = 0.0;
  imu_.orientation.y = 0.0;
  imu_.orientation.z = 0.0;
  imu_.orientation.w = 1.0;
  imu_.angular_velocity.x = 1.0;
  imu_.angular_velocity.y = 2.0;
  imu_.angular_velocity.z = 3.0;
  imu_.linear_acceleration.x = -1.0;
  imu_.linear_acceleration.y = -2.0;
  imu_.linear_acceleration.z = -3.0;
  imu_.orientation_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_.linear_acceleration_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  return imu_;
}

void SoloDriver::write_joint_eff_cmd(std::vector<double> eff_cmd)
{
  // Dummy
  for (size_t i = 0; i < joint_size_; i++) {
    // ROS_INFO("Joint %d Command: %lf", i, cmd.at(i));
  }
}
}  // namespace solo_driver
