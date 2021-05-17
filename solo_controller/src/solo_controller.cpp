// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_controller/solo_controller.hpp"

namespace solo_controller
{
// https://github.com/ros-controls/ros_controllers/blob/noetic-devel/four_wheel_steering_controller/src/four_wheel_steering_controller.cpp
bool SoloController::init(
  hardware_interface::EffortJointInterface * hw,
  ros::NodeHandle & root_nh,
  ros::NodeHandle & controller_nh)
{
  // TODO(JaehyunShim): Check if exception throwing is needed
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
  joint_cmd_sub_ =
    controller_nh.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>(
      "joint_cmd", 10, &SoloController::joint_cmd_callback, this);
  // TODO(JaehyunShim): Check why writeFromNonRT is required in init()
  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_cmd_buffer;
  joint_cmd_buffer.positions.resize(joint_size_);
  joint_cmd_buffer.velocities.resize(joint_size_);
  joint_cmd_buffer.efforts.resize(joint_size_);
  for (size_t i = 0; i < joint_size_; i++) {
    joint_cmd_buffer.positions[i] = 0.0;
    joint_cmd_buffer.velocities[i] = 0.0;
    joint_cmd_buffer.efforts[i] = 0.0;
  }
  joint_cmd_buffer_.writeFromNonRT(joint_cmd_buffer);

  return true;
}

void SoloController::starting(const ros::Time & /*time*/) {}

void SoloController::update(const ros::Time & time, const ros::Duration & period)
{
  // Get current time
  ros::Time curr_time = time;

  // Get joint data
  for (size_t i = 0; i < joint_size_; i++) {
    pos_curr_[i] = joint_handle_[i].getPosition();
    vel_curr_[i] = joint_handle_[i].getVelocity();
  }

  // TODO(Jaehyun): Rewrite this chunk of code if there is a more
  // clean and intuitive way to process the first loop
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
  // TODO(Jaehyun): Check what happens if there is data left in buffer
  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_cmd_buffer =
    *(joint_cmd_buffer_.readFromRT());
  for (size_t i = 0; i < joint_size_; i++) {
    // TODO(Jaehyun): Add lines to check if joint_name equals joint_cmd_buffer.name[i]
    pos_ref_[i] = joint_cmd_buffer.positions[i];
    vel_ref_[i] = joint_cmd_buffer.velocities[i];
    eff_ref_[i] = joint_cmd_buffer.efforts[i];
  }

  // TODO(Jaehyun): Interpolate received reference data if needed
  // Check the email sent from Vlad

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
    // Notice: eff_cmd will diverge if you don't give pos_ref, vel_ref, eff_ref.
    // joint_handle_[i].setCommand(0.0);
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
}

void SoloController::stopping(const ros::Time & /*time*/) {}

}  // namespace solo_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(solo_controller::SoloController, controller_interface::ControllerBase)
