// Copyright 2021 University of Edinburgh
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
  for (size_t i = 0; i < joint_size_; i++)
  {
    pos_.at(i) = 0.1;
    // ROS_INFO("Joint %d Position: %lf", i, pos_.at(i));
  }

  return pos_;
}

std::vector<double> SoloDriver::read_joint_vel()
{
  // Dummy
  for (size_t i = 0; i < joint_size_; i++)
  {
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
  imu_.orientation_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  imu_.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  imu_.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  return imu_;
}

void SoloDriver::write_joint_eff_cmd(std::vector<double> eff_cmd)
{
  // Dummy
  for (size_t i = 0; i < joint_size_; i++)
  {
    // ROS_INFO("Joint %d Command: %lf", i, cmd.at(i));
  }
}
}  // namespace solo_driver
