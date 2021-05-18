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
#include <urdf/model.h>

#include <memory>
#include <string>
#include <vector>

namespace solo_hw_interface
{
class SoloHwInterface : public hardware_interface::RobotHW
{
public:
  SoloHwInterface(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh) {}
  ~SoloHwInterface() {}

  bool init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh);
  void read();
  void write();

private:
  // Joint
  u_int8_t joint_size_;
  std::vector<std::string> joint_name_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> pos_cmd_;
  std::vector<double> vel_cmd_;
  std::vector<double> eff_cmd_;

  // Interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface pos_joint_interface_;
  hardware_interface::VelocityJointInterface vel_joint_interface_;
  hardware_interface::EffortJointInterface eff_joint_interface_;

  // TODO(JaehyunShim): Check if joint_limit_interface needs to be added
  // Reference: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/include/gazebo_ros_control/default_robot_hw_sim.h#L113

  // Handle
  std::vector<hardware_interface::JointStateHandle> joint_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_command_handle_;

  // Driver
  std::shared_ptr<solo_driver::SoloDriver> solo_driver_;
};
}  // namespace solo_hw_interface
#endif  // SOLO_HW_INTERFACE__SOLO_HW_INTERFACE_HPP_
