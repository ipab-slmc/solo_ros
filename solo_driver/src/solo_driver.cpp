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

#include "master_board_sdk/master_board_interface.h"
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/imu.hpp"
#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

namespace solo_driver
{
void SoloDriver::init(uint8_t joint_size)
{
  joint_size_ = joint_size;
  pos_.resize(joint_size_);
  vel_.resize(joint_size_);

  // Ref:
  // https://github.com/stack-of-tasks/ros2_control_bolt/blob/master/ros2_hardware_interface_bolt/src/system_bolt.cpp#L53
  // Get the ethernet interface to discuss with the ODRI master board
  // eth_interface_ = info_.hardware_parameters.at("eth_interface");
  // TODO(Jae): Ethernet
  master_board_ = std::make_shared<MasterBoardInterface>(eth_interface_);

  auto imu = std::make_shared<odri_control_interface::IMU>(master_board_);

  // Define joints (ODRI)
  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // Motor numbers
  //   motor_numbers_[joint_name_to_motor_nb_[joint.name]] = stoi(joint.parameters.at("motor_number"));
  //   // Reversed polarities
  //   if (joint.parameters.at("motor_reversed_polarity") == "true")
  //   {
  //     motor_reversed_polarities_[joint_name_to_motor_nb_[joint.name]] = true;
  //   }
  //   else
  //   {
  //     motor_reversed_polarities_[joint_name_to_motor_nb_[joint.name]] = false;
  //   }
  //   // Joint parameters
  //   joint_lower_limits_[joint_name_to_motor_nb_[joint.name]] =
  //       stod(joint.command_interfaces[0].min);  // Modif d'après lecture des capteurs (demo bolt)
  //   joint_upper_limits_[joint_name_to_motor_nb_[joint.name]] =
  //       stod(joint.command_interfaces[0].max);  // Modif d'après lecture des capteurs (demo bolt)

  //   motor_constants_ = stod(joint.parameters.at("motor_constant"));
  //   gear_ratios_ = stod(joint.parameters.at("gear_ratio"));
  //   max_currents_ = stod(joint.parameters.at("max_current"));
  //   max_joint_velocities_ = stod(joint.parameters.at("max_joint_velocity"));
  //   safety_damping_ = stod(joint.parameters.at("safety_damping"));
  // }

  // joint_ = std::make_shared<odri_control_interface::JointModules>(
  //     master_board_, motor_numbers_, motor_constants_, gear_ratios_, max_currents_,
  //     motor_reversed_polarities_, joint_lower_limits_, joint_upper_limits_, max_joint_velocities_,
  //     safety_damping_);

  // robot_ = std::make_shared<odri_control_interface::Robot>(master_board_, joint_, imu);

  robot_->Start();
}

std::vector<double> SoloDriver::read_joint_pos()
{
  // Dummy
  robot_->ParseSensorData();
  auto pos = robot_->joints->GetPositions();
  auto vel = robot_->joints->GetVelocities();
  auto eff = robot_->joints->GetMeasuredTorques();
  for (size_t i = 0; i < joint_size_; i++)
  {
    pos_.at(i) = 0.1;
    // ROS_INFO("Joint %d Position: %lf", i, pos_.at(i));
    pos_.at(i) = pos[i];
    vel_.at(i) = vel[i];
    eff_.at(i) = eff[i];
  }

  return pos_;
}

std::vector<double> SoloDriver::read_joint_vel()
{
  // Dummy
  robot_->ParseSensorData();
  auto pos = robot_->joints->GetPositions();
  auto vel = robot_->joints->GetVelocities();
  auto eff = robot_->joints->GetMeasuredTorques();
  for (size_t i = 0; i < joint_size_; i++)
  {
    vel_.at(i) = 0.2;
    // ROS_INFO("Joint %d Velocity: %lf", i, vel_.at(i));
    pos_.at(i) = pos[i];
    vel_.at(i) = vel[i];
    eff_.at(i) = eff[i];
  }

  return vel_;
}

sensor_msgs::Imu SoloDriver::read_imu()
{
  robot_->ParseSensorData();
  auto ori = robot_->imu->GetAttitudeQuaternion();
  auto ang_vel = robot_->imu->GetGyroscope();
  auto lin_acc = robot_->imu->GetLinearAcceleration();

  imu_.orientation.x = ori[0];
  imu_.orientation.y = ori[1];
  imu_.orientation.z = ori[2];
  imu_.orientation.w = ori[3];
  imu_.angular_velocity.x = ang_vel[0];
  imu_.angular_velocity.y = ang_vel[1];
  imu_.angular_velocity.z = ang_vel[2];
  imu_.linear_acceleration.x = lin_acc[0];
  imu_.linear_acceleration.y = lin_acc[1];
  imu_.linear_acceleration.z = lin_acc[2];
  imu_.orientation_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  imu_.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  imu_.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  return imu_;
}

void SoloDriver::write_joint_eff_cmd(std::vector<double> eff_cmd)
{
  for (size_t i = 0; i < joint_size_; i++)
  {
    eff_cmd_[i] = eff_cmd.at(i);
    // ROS_INFO("Joint %d Command: %lf", i, cmd.at(i));
  }
  // robot_->joints->SetDesiredPositions(pos_cmd_);
  // robot_->joints->SetDesiredVelocities(vel_cmd_);
  robot_->joints->SetTorques(eff_cmd_);
  // robot_->joints->SetPositionGains(kp_);
  // robot_->joints->SetVelocityGains(kd_);
}
}  // namespace solo_driver
