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
bool SoloHwInterface::init(ros::NodeHandle & root_nh, ros::NodeHandle & robot_hw_nh)
{
  // Get joint names from URDF
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", root_nh)) {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  for(std::map<std::string, std::shared_ptr<urdf::Joint>>::iterator i = urdf.joints_.begin(); i != urdf.joints_.end(); i++) {
    std::shared_ptr<urdf::Joint> joint_urdf = i->second;
    if (joint_urdf->type == urdf::Joint::REVOLUTE) {
      joint_name_.emplace_back(i->first);
    }
  }

  joint_size_ = joint_name_.size();
  for (size_t i = 0; i < joint_size_; i++) {
    printf("Joint name: %s \n", joint_name_[i].c_str());
  }
  pos_.resize(joint_size_);
  vel_.resize(joint_size_);
  eff_.resize(joint_size_);
  pos_cmd_.resize(joint_size_);
  vel_cmd_.resize(joint_size_);
  eff_cmd_.resize(joint_size_);

  // Init robot driver
  solo_driver_ = std::make_shared<solo_driver::SoloDriver>();
  solo_driver_->init(joint_size_);

  // TODO(JaehyunShim): Reference
  // Reference: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp
  for (size_t i = 0; i < joint_size_; i++) {
    // Register joint state handle
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(
        joint_name_.at(i), &pos_.at(i), &vel_.at(i), &eff_.at(i)));

    // Register joint position command handle
    pos_joint_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_name_.at(i)), &pos_cmd_.at(i)));

    // Register joint velocity command handle
    vel_joint_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_name_.at(i)), &vel_cmd_.at(i)));

    // Register joint effort command handle
    eff_joint_interface_.registerHandle(
      hardware_interface::JointHandle(
        joint_state_interface_.getHandle(joint_name_.at(i)), &eff_cmd_.at(i)));
  }

  // Register joint interfaces
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
  // double data;
  // data = solo_driver_->read_sensor();
}

void SoloHwInterface::write()
{
  // TODO(JaehyunShim): Add a feature for switching between pos, vel, eff
  // Should not use std::move() as eff_cmd_ should not be removed
  solo_driver_->write_joint(eff_cmd_);  // maybe better name it to write_joint_cmd
}

}  // namespace solo_hw_interface
