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

#ifndef SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_
#define SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_

#include <controller_manager/controller_manager.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <solo_driver/solo_driver.hpp>
#include <urdf/model.h>

#include <memory>
#include <string>
#include <vector>

namespace solo_hw_interface
{
class SoloHwInterface : public hardware_interface::RobotHW
{
public:
  SoloHwInterface(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh)
  {
  }
  ~SoloHwInterface()
  {
  }

  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh);
  void read();
  void write();

private:
  // Joint
  u_int8_t joint_size_;
  std::vector<std::string> joint_name_;
  std::vector<double> pos_;      // Position
  std::vector<double> vel_;      // Velocity
  std::vector<double> eff_;      // Effort
  std::vector<double> pos_cmd_;  // Position Command
  std::vector<double> vel_cmd_;  // Velocity Command
  std::vector<double> eff_cmd_;  // Effort Command

  // IMU
  double ori_[4];          // Orientation
  double ori_cov_[9];      // Orientation Covariance
  double ang_vel_[3];      // Angular Velocity
  double ang_vel_cov_[9];  // Angular Velocity Covariance
  double lin_acc_[3];      // Linear Acceleration
  double lin_acc_cov_[9];  // Linear Acceleration Covariance
  sensor_msgs::Imu imu_;

  // Interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface pos_joint_interface_;
  hardware_interface::VelocityJointInterface vel_joint_interface_;
  hardware_interface::EffortJointInterface eff_joint_interface_;
  hardware_interface::ImuSensorInterface imu_interface_;

  // TODO(JaehyunShim): Check if joint_limit_interface needs to be added
  // Reference:
  // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/include/gazebo_ros_control/default_robot_hw_sim.h#L113

  // Handle
  std::vector<hardware_interface::JointStateHandle> joint_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_command_handle_;

  // Driver
  std::shared_ptr<solo_driver::SoloDriver> solo_driver_;
};
}  // namespace solo_hw_interface
#endif  // SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_
