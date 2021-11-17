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

// Reference: https://github.com/open-dynamic-robot-initiative/solo

#ifndef SOLO_DRIVER__SOLO_DRIVER_HPP_
#define SOLO_DRIVER__SOLO_DRIVER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <vector>

#include "master_board_sdk/master_board_interface.h"
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/imu.hpp"
#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

namespace solo_driver
{
class SoloDriver
{
public:
  SoloDriver()
  {
  }
  ~SoloDriver()
  {
  }

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
  std::vector<double> eff_;

  Eigen::VectorXd eff_cmd_;

  // IMU
  sensor_msgs::Imu imu_;

  // Robot
  std::string eth_interface_;

  Eigen::VectorXi motor_numbers_;
  Eigen::VectorXd motor_reversed_polarities_;
  Eigen::VectorXd joint_lower_limits_;
  Eigen::VectorXd joint_upper_limits_;
  Eigen::VectorXd position_offsets_;

  double motor_constants_;
  double gear_ratios_;
  double max_currents_;
  double max_joint_velocities_;
  double safety_damping_;

  std::shared_ptr<odri_control_interface::Robot> robot_;
  std::shared_ptr<odri_control_interface::JointModules> joint_;
  std::shared_ptr<MasterBoardInterface> master_board_;
};

}  // namespace solo_driver
#endif  // SOLO_DRIVER__SOLO_DRIVER_HPP_
