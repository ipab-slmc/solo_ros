// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_
#define SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <solo_driver/solo_driver.hpp>

#include <memory>
#include <string>
#include <vector>

namespace solo_hw_interface
{
class SoloHwInterface : public hardware_interface::RobotHW
{
public:
  SoloHwInterface(ros::NodeHandle root_nh, ros::NodeHandle controller_nh);
  ~SoloHwInterface() {}

  void read();  // hardware_interface::return_type ??
  void write();

private:
  // Node Handle
  ros::NodeHandle root_nh_;
  ros::NodeHandle controller_nh_;

  // Joint
  u_int8_t joint_size_;
  std::vector<std::string> joint_name_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> cmd_;

  // Interface
  std::vector<hardware_interface::JointStateInterface> joint_state_interface_;
  std::vector<hardware_interface::EffortJointInterface> joint_command_interface_;
  // std::vector<hardware_interface::PositionJointInterface> pos_joint_interface_;
  // std::vector<hardware_interface::VelocityJointInterface> vel_joint_interface_;
  // std::vector<hardware_interface::EffortJointInterface> eff_joint_interface_;

  // Handle
  std::vector<hardware_interface::JointStateHandle> joint_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_command_handle_;

  // Driver
  std::shared_ptr<solo_hw_interface::SoloDriver> solo_driver_;
};
}  // namespace solo_hw_interface
#endif  // SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_
