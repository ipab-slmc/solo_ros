// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>

#include "solo_controller/solo_controller.hpp"

namespace solo_controller
{
bool SoloController::init(
  hardware_interface::EffortJointInterface * hw,
  ros::NodeHandle & root_nh,
  ros::NodeHandle & controller_nh)
{
  // TODO(JaehyunShim): Add exception throwing lines
  controller_nh.getParam("joint", joint_name_);
  joint_size_ = joint_name_.size();

  // Resize joint data vectors
  for (size_t i = 0; i < joint_size_; i++) {
    kp_.emplace_back(0.0);
    kd_.emplace_back(0.0);
    pos_curr_.emplace_back(0.0);
    vel_curr_.emplace_back(0.0);
    pos_prev_.emplace_back(0.0);
    vel_prev_.emplace_back(0.0);
    pos_ref_.emplace_back(0.0);
    vel_ref_.emplace_back(0.0);
    eff_ref_.emplace_back(0.0);
    eff_cmd_.emplace_back(0.0);
  }

  // Get PD gains from parameter server
  for (size_t i = 0; i < joint_size_; i++) {
    controller_nh.getParam("gain/" + joint_name_[i] + "/pid/p", kp_[i]);
    controller_nh.getParam("gain/" + joint_name_[i] + "/pid/d", kd_[i]);
  }

  // Joint Handle
  for (size_t i = 0; i < joint_size_; i++) {
    try {
      joint_handle_.emplace_back(hw->getHandle(joint_name_[i]));
    } catch (const hardware_interface::HardwareInterfaceException & e) {
      ROS_ERROR_STREAM("Error: " << e.what());
      return false;
    }
  }

  // Initialize ROS publishers
  // TODO(JaehyunShim): Need more consideration on the queue size
  rt_joint_state_pub_.reset(
    new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
      controller_nh, "joint_states", 10));

  // Initialize ROS subscribers
  // TODO(JaehyunShim): Need more consideration on the queue size
  joint_command_sub_ =
    controller_nh.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>(
    "joint_command", 10, &SoloController::joint_command_callback, this);

  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_command_buffer;
  joint_command_buffer.positions.resize(joint_size_);
  joint_command_buffer.velocities.resize(joint_size_);
  joint_command_buffer.efforts.resize(joint_size_);
  for (size_t i = 0; i < joint_size_; i++) {
    joint_command_buffer.positions[i] = 0.0;
    joint_command_buffer.velocities[i] = 0.0;
    joint_command_buffer.efforts[i] = 0.0;
  }
  joint_command_buffer_.writeFromNonRT(joint_command_buffer);

  // Dynamic reconfigure for PD gains
  // Reference: https://github.com/ros-controls/control_toolbox/blob/melodic-devel/src/pid.cpp
  for (size_t i = 0; i < joint_size_; i++) {
    std::string dyn_reconf_nh_ns =
      controller_nh.getNamespace() + "/gain/" + joint_name_[i] + "/pid";
    ros::NodeHandle dyn_reconf_nh(dyn_reconf_nh_ns);
    dyn_reconf_server_.emplace_back(
      std::make_shared<dynamic_reconfigure::Server<solo_controller::SoloControllerConfig>>(
        dyn_reconf_mutex_, dyn_reconf_nh));
  }
  solo_controller_config_.resize(joint_size_);

  solo_controller::SoloControllerConfig solo_controller_config;
  for (size_t i = 0; i < joint_size_; i++) {
    // Set init param values
    solo_controller_config.p = kp_[i];
    solo_controller_config.d = kd_[i];
    dyn_reconf_mutex_.lock();
    dyn_reconf_server_[i]->updateConfig(solo_controller_config);
    dyn_reconf_mutex_.unlock();

    // Set server callback
    dyn_reconf_server_[i]->setCallback(
      boost::bind(
        &SoloController::dyn_reconf_callback, this, _1, _2, i));
  }

  // URDF parsing for enforcing joint limit
  urdf::Model urdf;
  ros::NodeHandle urdf_nh("");
  if (!urdf.initParamWithNodeHandle("robot_description", urdf_nh)) {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  // TODO(JaehyunShim): Add exception throwing lines
  for (size_t i = 0; i < joint_size_; i++) {
    joint_urdf_.emplace_back(urdf.getJoint(joint_name_[i]));
  }

  return true;
}

void SoloController::starting(const ros::Time & /*time*/) {}

void SoloController::update(const ros::Time & time, const ros::Duration & period)
{
  // Get current time
  ros::Time curr_time = time;
  // TODO(Jaehyun): Check why sometimes time becomes negative
  // ROS_INFO("Current Time: %d", time.toNSec());

  // Get joint data
  for (size_t i = 0; i < joint_size_; i++) {
    pos_curr_[i] = joint_handle_[i].getPosition();
    vel_curr_[i] = joint_handle_[i].getVelocity();
  }

  // TODO(Jaehyun): Rewrite this chunk of code if there is a more
  // clean and intuitive way to process the first loop.
  // First loop for getting previous position and velocity
  if (!update_onoff) {
    for (size_t i = 0; i < joint_size_; i++) {
      pos_prev_[i] = pos_curr_[i];
      vel_prev_[i] = vel_curr_[i];
    }
    update_onoff = true;
    return;
  }

  // Get reference position, velocity, effort from the planner
  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_command_buffer =
    *(joint_command_buffer_.readFromRT());
  for (size_t i = 0; i < joint_size_; i++) {
    // TODO(Jaehyun): Add lines checking if joint_name equals joint_command_buffer.name[i]
    pos_ref_[i] = joint_command_buffer.positions[i];
    vel_ref_[i] = joint_command_buffer.velocities[i];
    eff_ref_[i] = joint_command_buffer.efforts[i];
  }

  // TODO(Jaehyun): Interpolate received reference data if needed

  // Compute effort command
  for (size_t i = 0; i < joint_size_; i++) {
    eff_cmd_[i] = eff_ref_[i] +
      kp_[i] * (vel_ref_[i] - vel_prev_[i]) +
      kd_[i] * (pos_ref_[i] - pos_prev_[i]);
  }

  // Save previous position and velocity
  for (size_t i = 0; i < joint_size_; i++) {
    pos_prev_[i] = pos_curr_[i];
    vel_prev_[i] = vel_curr_[i];
  }

  // Send effort command to motors
  for (size_t i = 0; i < joint_size_; i++) {
    enforce_joint_limit(eff_cmd_[i], i);
    joint_handle_[i].setCommand(eff_cmd_[i]);
    // ROS_INFO("%d joint command: %lf", i, eff_cmd_[i]);
  }

  // Publish joint_state data
  if (rt_joint_state_pub_->trylock()) {
    rt_joint_state_pub_->msg_.header.stamp = curr_time;
    rt_joint_state_pub_->msg_.name = joint_name_;
    rt_joint_state_pub_->msg_.position = pos_curr_;
    rt_joint_state_pub_->msg_.velocity = vel_curr_;
    rt_joint_state_pub_->unlockAndPublish();
  }

  // Update parameters
  for (size_t i = 0; i < joint_size_; i++) {
    solo_controller::SoloControllerConfig solo_controller_config =
      *(solo_controller_config_[i].readFromRT());
    kp_[i] = solo_controller_config.p;
    kd_[i] = solo_controller_config.d;
    // ROS_INFO("%d joint kp_: %lf", i, kp_[i]);
    // ROS_INFO("%d joint kd_: %lf", i, kd_[i]);
  }
}

void SoloController::stopping(const ros::Time & /*time*/) {}

}  // namespace solo_controller

PLUGINLIB_EXPORT_CLASS(solo_controller::SoloController, controller_interface::ControllerBase)
