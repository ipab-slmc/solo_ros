// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

// Reference: https://github.com/open-dynamic-robot-initiative/solo

#ifndef SOLO_DRIVER__SOLO_DRIVER_HPP_
#define SOLO_DRIVER__SOLO_DRIVER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <vector>

namespace solo_driver
{
class SoloDriver
{
public:
  SoloDriver() {}
  ~SoloDriver() {}

  void init(uint8_t joint_size);

  // Joint
  std::vector<double> read_joint_pos();
  std::vector<double> read_joint_vel();
  void write_joint_eff_cmd(std::vector<double> eff_cmd);

  // IMU
  sensor_msgs::Imu read_imu();

private:
  // Joint
  uint8_t joint_size_;
  std::vector<double> pos_;
  std::vector<double> vel_;

  // IMU
  sensor_msgs::Imu imu_;
};

}  // namespace solo_driver
#endif  // SOLO_DRIVER__SOLO_DRIVER_HPP_
