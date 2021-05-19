// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <map>
#include <memory>
#include <string>
#include <utility>

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

  for (std::map<std::string, std::shared_ptr<urdf::Joint>>::iterator i = urdf.joints_.begin();
    i != urdf.joints_.end(); i++)
  {
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
  // Register imu handle
  // TODO(JaehyunShim): If needed, allow users to set name, frame_id at runtime
  imu_interface_.registerHandle(
    hardware_interface::ImuSensorHandle(
      "imu", "base_link", ori_, ori_cov_, ang_vel_, ang_vel_cov_, lin_acc_, lin_acc_cov_));

  // Register joint interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&pos_joint_interface_);
  registerInterface(&vel_joint_interface_);
  registerInterface(&eff_joint_interface_);
  registerInterface(&imu_interface_);

  return true;
}

void SoloHwInterface::read()
{
  // Joint State
  pos_ = std::move(solo_driver_->read_joint_pos());  // maybe better name it to read_joint_state
  vel_ = std::move(solo_driver_->read_joint_vel());  // maybe better name it to read_joint_state

  // IMU
  imu_ = solo_driver_->read_imu();
  ori_[0] = imu_.orientation.x;
  ori_[1] = imu_.orientation.y;
  ori_[2] = imu_.orientation.z;
  ori_[3] = imu_.orientation.w;
  ang_vel_[0] = imu_.angular_velocity.x;
  ang_vel_[1] = imu_.angular_velocity.y;
  ang_vel_[2] = imu_.angular_velocity.z;
  lin_acc_[0] = imu_.linear_acceleration.x;
  lin_acc_[1] = imu_.linear_acceleration.y;
  lin_acc_[2] = imu_.linear_acceleration.z;
  for (size_t i = 0; i < 9; i++) {
    ori_cov_[i] = imu_.orientation_covariance[i];
    ang_vel_cov_[i] = imu_.angular_velocity_covariance[i];
    lin_acc_cov_[i] = imu_.linear_acceleration_covariance[i];
  }
}

void SoloHwInterface::write()
{
  // Joint Command
  solo_driver_->write_joint_eff_cmd(eff_cmd_);
}

}  // namespace solo_hw_interface
