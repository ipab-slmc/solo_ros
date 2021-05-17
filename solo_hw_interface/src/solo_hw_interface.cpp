// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <memory>

#include "solo_hw_interface/solo_hw_interface.hpp"

namespace solo_hw_interface
{
SoloHwInterface::SoloHwInterface(ros::NodeHandle root_nh, ros::NodeHandle controller_nh)
: root_nh_(root_nh),
  controller_nh_(controller_nh),
  solo_driver_(std::make_shared<solo_driver::SoloDriver>())
{
  // Get joint names
  // TODO(JaehyunShim): Add urdf parsing (check the reference)
  joint_name_.emplace_back("Joint");
  joint_size_ = joint_name_.size();
  solo_driver_->init(joint_size_);
}

bool SoloHwInterface::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle & /*robot_hw_nh*/)
{
  // https://github.com/ROBOTIS-SYS/aimbot_base/blob/d32fe11eac74f0f91f8ba814ab9983e253e4833c/aimbot_hw/src/hardware_interface.cpp
  joint_state_interface_.resize(joint_size_);
  pos_joint_interface_.resize(joint_size_);
  vel_joint_interface_.resize(joint_size_);
  eff_joint_interface_.resize(joint_size_);
  pos_.resize(joint_size_);
  vel_.resize(joint_size_);
  eff_.resize(joint_size_);
  pos_cmd_.resize(joint_size_);
  vel_cmd_.resize(joint_size_);
  eff_cmd_.resize(joint_size_);

  // TODO(JaehyunShim): Reference
  // Reference: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp
  for (size_t i = 0; i < joint_size_; i++) {
    // Joint State
    joint_state_interface_.at(i).registerHandle(
      hardware_interface::JointStateHandle(
        joint_name_.at(i), &pos_.at(i), &vel_.at(i), &eff_.at(i)));

    // Joint Position Command
    pos_joint_interface_.at(i).registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.at(i).getHandle(joint_name_.at(i)), &pos_cmd_.at(i)));

    // Joint Velocity Command
    vel_joint_interface_.at(i).registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.at(i).getHandle(joint_name_.at(i)), &vel_cmd_.at(i)));

    // Joint Effort Command
    eff_joint_interface_.at(i).registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.at(i).getHandle(joint_name_.at(i)), &eff_cmd_.at(i)));
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&pos_joint_interface_);
  registerInterface(&vel_joint_interface_);
  registerInterface(&eff_joint_interface_);

  return true;
}

void SoloHwInterface::read()
{
  pos_ = std::move(solo_driver_->read_joint());  // maybe better name it to read_joint_state

  // TODO(JaehyunShim): For IMU, should move to imu_hw_interface later
  double data;
  data = solo_driver_->read_sensor();
}

void SoloHwInterface::write()
{
  // TODO(JaehyunShim): Add a feature for switching between pos, vel, eff
  solo_driver_->write_joint(std::move(eff_cmd_));  // maybe better name it to write_joint_cmd
}

}  // namespace solo_hw_interface
