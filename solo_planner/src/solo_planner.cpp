// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <string>
#include <vector>

#include "solo_planner/solo_planner.hpp"

namespace solo_planner
{
SoloPlanner::SoloPlanner()
: nh_("")
{
  // Get joint names
  // TODO(JaehyunShim): Check how to get joint names from planner lib
  // If no other way, use URDF::Model
  std::vector<std::string> joint_name_;
  // Temp code
  joint_name_.emplace_back("FL_HAA");
  joint_name_.emplace_back("FL_HFE");
  joint_name_.emplace_back("FL_KFE");
  joint_name_.emplace_back("FR_HAA");
  joint_name_.emplace_back("FR_HFE");
  joint_name_.emplace_back("FR_KFE");
  joint_name_.emplace_back("HL_HAA");
  joint_name_.emplace_back("HL_HFE");
  joint_name_.emplace_back("HL_KFE");
  joint_name_.emplace_back("HR_HAA");
  joint_name_.emplace_back("HR_HFE");
  joint_name_.emplace_back("HR_KFE");
  joint_size_ = joint_name_.size();

  for (size_t i = 0; i < joint_size_; i++) {
    kp_.emplace_back(0.0);
    kd_.emplace_back(0.0);
    pos_curr_.emplace_back(0.0);
    vel_curr_.emplace_back(0.0);
    pos_ref_.emplace_back(0.0);
    vel_ref_.emplace_back(0.0);
    eff_ref_.emplace_back(0.0);
  }

  // Initialize ROS publishers
  // TODO(JaehyunShim): Need more consideration on the queue size
  rt_joint_command_pub_.reset(
    new realtime_tools::RealtimePublisher<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>(
      nh_, "/solo_controller/joint_command", 10));

  // Initialize ROS subscribers
  // TODO(JaehyunShim): Need more consideration on the queue size
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
    "/solo_controller/joint_states", 10, &SoloPlanner::joint_state_callback, this);
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
    "imu", 10, &SoloPlanner::imu_callback, this);

  // TODO(JaehyunShim): Consider if just emplace_back will be better
  sensor_msgs::JointState joint_state_buffer;
  joint_state_buffer.name.resize(joint_size_);
  joint_state_buffer.position.resize(joint_size_);
  joint_state_buffer.velocity.resize(joint_size_);
  joint_state_buffer.effort.resize(joint_size_);
  for (size_t i = 0; i < joint_size_; i++) {
    joint_state_buffer.name[i] = joint_name_[i];
    joint_state_buffer.position[i] = 0.0;
    joint_state_buffer.velocity[i] = 0.0;
  }
  joint_state_buffer_.writeFromNonRT(joint_state_buffer);

  sensor_msgs::Imu imu_buffer;
  imu_buffer.orientation.x = 0.0;
  imu_buffer.orientation.y = 0.0;
  imu_buffer.orientation.z = 0.0;
  imu_buffer.orientation.w = 0.0;
  imu_buffer.angular_velocity.x = 0.0;
  imu_buffer.angular_velocity.y = 0.0;
  imu_buffer.angular_velocity.z = 0.0;
  imu_buffer.linear_acceleration.x = 0.0;
  imu_buffer.linear_acceleration.y = 0.0;
  imu_buffer.linear_acceleration.z = 0.0;
  imu_buffer_.writeFromNonRT(imu_buffer);

  // Initialize ROS timer
  // TODO(JaehyunShim): Is it real-time safe to use ros timer?
  timer_ = nh_.createTimer(ros::Duration(0.001), &SoloPlanner::timer_callback, this);
}

void SoloPlanner::timer_callback(const ros::TimerEvent & te)
{
  // Get current position, velocity from the solo_controller
  sensor_msgs::JointState joint_state_buffer =
    *(joint_state_buffer_.readFromRT());
  for (size_t i = 0; i < joint_size_; i++) {
    pos_curr_[i] = joint_state_buffer.position[i];
    vel_curr_[i] = joint_state_buffer.velocity[i];
  }

  // Get current imu data from imu_sensor_controller
  sensor_msgs::Imu imu_buffer = *(imu_buffer_.readFromRT());
  imu_ = imu_buffer;

  // Planning
  // ------------------------------------------------------------------------------------------
  // TODO(JaehyunShim): Use compute_references function from the quadruped-reactive-walking lib
  // Input: pos_curr_, vel_curr_, imu_
  // Output: pos_ref_, vel_ref_, eff_ref_, kp_, kd_
  // {pos_ref_, vel_ref_, eff_ref_, kp_, kd_} = compute_references(pos_curr_, vel_curr_, imu_)
  // ------------------------------------------------------------------------------------------

  // Publish joint_command data
  if (rt_joint_command_pub_->trylock()) {
    // TODO(JaehyunShim): See how readFromRT works.
    // If time stamp is used in the process, add timestamp to ipab_controller_msgs
    rt_joint_command_pub_->msg_.name = joint_name_;
    rt_joint_command_pub_->msg_.positions = pos_ref_;
    rt_joint_command_pub_->msg_.velocities = vel_ref_;
    rt_joint_command_pub_->msg_.efforts = eff_ref_;
    // TODO(JaehyunShim): Check how to kp, kd are going to be sent
    // rt_joint_command_pub_->msg_.position_gains = kp_;
    // rt_joint_command_pub_->msg_.velocity_gains = kd_;
    rt_joint_command_pub_->unlockAndPublish();
  }
}
}  // namespace solo_planner
