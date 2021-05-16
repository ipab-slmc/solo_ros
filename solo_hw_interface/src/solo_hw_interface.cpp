// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_hw_interface/solo_hw_interface.hpp"

namespace solo_hw_interface
{
SoloHwInterface::SoloHwInterface(ros::NodeHandle root_nh, ros::NodeHandle controller_nh)
: root_nh_(root_nh),
  controller_nh_(controller_nh),
  solo_driver_(std::make_shared<solo_hw_interface::SoloDriver>())
{
  // Get joint names
  joint_name_.emplace_back("left_wheel_joint",);
  joint_name_.emplace_back("right_wheel_joint");
  joint_size_ = joint_name_.size();
  solo_driver_->init(joint_size_);

  // ??
  // https://github.com/ROBOTIS-SYS/aimbot_base/blob/d32fe11eac74f0f91f8ba814ab9983e253e4833c/aimbot_hw/src/hardware_interface.cpp
  joint_state_interface_.resize(joint_size_);
  pos_joint_interface_.resize(joint_size_);
  vel_joint_interface_.resize(joint_size_);
  eff_joint_interface_.resize(joint_size_);

  pos_.assign(joint_size_, 0.0);
  vel_.assign(joint_size_, 0.0);
  eff_.assign(joint_size_, 0.0);
  cmd_.assign(joint_size_, 0.0);

  for (uint8_t i = 0; i < joint_size_; i++) {
    // Joint State
    joint_state_handle_.at(i) = hardware_interface::JointStateHandle(
      joint_name_.at(i), &pos_.at(i), &vel_.at(i), &eff_.at(i));
    ret = joint_state_interface_.registerHandle(&joint_state_handle_.at(i));
    // if (ret != hardware_interface::return_type::OK) { // exist???????
    if (ret != hardware_interface::return_type::OK) {
      ROS_INFO("can't register joint state handle %s", joint_name_.at(i).c_str());
      return ret;
    }

    // Joint Command
    joint_handle.at(i) = hardware_interface::JointHandle(
      joint_name_.at(i), &cmd_.at(i));
    ret = joint_interface_.registerHandle(&joint_handle_.at(i));
    // if (ret != hardware_interface::return_type::OK) { // exist???????
    if (ret != hardware_interface::return_type::OK) {
      ROS_INFO("can't register joint handle %s", joint_name_.at(i).c_str());
      return ret;
    }
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_interface_);




  Action action = Action::Zero();
  while (1) {
    auto t = frontend.append_desired_action(action);
    auto first_observation = frontend.get_observation(t);
  }

  return true;
}

void SoloHwInterface::read()
{
  static double pos[joint_size_];
  pos = solo_driver_->read_joint();  // maybe better name it to read_joint_state
  for (uint8_t i = 0; i < joint_size_; i++) {
    // Read
    // pos_.at(i) = pos[i];
    printf("position for joint %d: %lf \n", i, pos_.at(i));
  }

  double data;
  data = solo_driver_->read_sensor();
  printf("sensor data: %lf \n", data);
}

void SoloHwInterface::write()
{
  static double cmd[joint_size_];
  for (uint8_t i = 0; i < joint_size_; i++) {
    // Write
    // cmd[i] = cmd_.at(i);
    printf("command for joint %d: %lf \n", i, cmd_.at(i));
  }
  solo_driver_->write_joint(cmd);  // maybe better name it to write_joint_cmd
}

} // namespace solo_hw
