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
bool SoloController::init(hardware_interface::RobotHW * robot_hw,
  ros::NodeHandle & root_nh,
  ros::NodeHandle & controller_nh)
{
  // Joint HWInterface
  hardware_interface::PositionJointInterface *const pos_joint_hw = robot_hw->get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface *const vel_joint_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface *const eff_joint_hw = robot_hw->get<hardware_interface::EffortJointInterface>();

  // IMU HWInterface
  // https://github.com/PR2/pr2_common/tree/melodic-devel/pr2_description/urdf/sensors
  // TODO(JaehyunShim): Get IMU data using the ros_control framework
  hardware_interface::ImuSensorInterface *const imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();

  // TODO(JaehyunShim): Check if exception throwing is needed
  controller_nh.getParam("joint", joint_name_);
  joint_size_ = joint_name_.size();
  for (size_t i = 0; i < joint_size_; i++) {
    controller_nh.getParam("gain/" + joint_names_[i] + "/pid/p", kp_[i]);
    controller_nh.getParam("gain/" + joint_names_[i] + "/pid/d", kd_[i]);
  }
  controller_nh.getParam("imu", imu_name_);

  // Joint Handle
  for (size_t i = 0; i < joint_size_; i++) {
    try {
      joint_handle_.push_back(eff_joint_hw->getHandle(joint_name_[i]));
    } catch (const hardware_interface::HardwareInterfaceException &e) {
      ROS_ERROR_STREAM("Error: " << e.what());
      return false;
    }
  }

  // IMU Handle
  // TODO(JaehyunShim): Check if multiple imu sensors should be considered. If yes, change to multiple (same as joint)
  try {
    imu_handle_.push_back(imu_hw->getHandle(imu_name_));
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR_STREAM("Error: " << e.what());
    return false;
  }

  // Initialize ROS publishers
  // TODO(JaehyunShim): Need more consideration on the queue size
  rt_joint_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(controller_nh, "joint_states", 10));
  rt_imu_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::Imu>(controller_nh, "imu", 10));

  // Initialize ROS subscribers
  // TODO(JaehyunShim): Need more consideration on the queue size
  joint_cmd_sub_ = controller_nh.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory>(
    "joint_cmd", 10, &SoloController::joint_cmd_callback, this);

  return true;
}

void SoloController::starting(const ros::Time & /*time*/) {}

void SoloController::update(const ros::Time & time, const ros::Duration & period)
{
  // Get current time
  ros::Time curr_time = time;

  // Get joint data
  double pos_curr[joint_size_];
  double vel_curr[joint_size_];
  for (size_t i = 0; i < joint_size_; i++) {
    pos_curr[i] = joint_handle_[i].getPosition();
    vel_curr[i] = joint_handle_[i].getVelocity();
  }

  // TODO(Jaehyun): Rewrite this chunk of code if there is a more clean and intuitive way to process the first loop
  // First loop for getting previous position and velocity
  static double pos_prev[joint_size_];
  static double vel_prev[joint_size_];
  if (!update_onoff)
  {
    for (size_t i = 0; i < joint_size_; i++) {
      pos_prev[i] = pos_curr[i];
      vel_prev[i] = vel_curr[i];
    }
    update_onoff = true;
    return;
  }

  // Get IMU data
  // TODO(Jaehyun): Add more imu data if needed
  // Only added angular velocity and linear acceleration without much background knowledge
  // after checking Fig2 from https://arxiv.org/pdf/1909.06586.pdf
  // Useful future reference: https://github.com/ros-controls/ros_controllers/blob/noetic-devel/imu_sensor_controller/src/imu_sensor_controller.cpp
  geometry_msgs::Vector3 ang_vel;
  ang_vel.x = imu_handle_[i].getAngularVelocity()[0];
  ang_vel.y = imu_handle_[i].getAngularVelocity()[1];
  ang_vel.z = imu_handle_[i].getAngularVelocity()[2];

  geometry_msgs::Vector3 lin_acc;
  lin_acc.x = imu_handle_[i].getLinearAcceleration()[0];
  lin_acc.y = imu_handle_[i].getLinearAcceleration()[1];
  lin_acc.z = imu_handle_[i].getLinearAcceleration()[2];

  // Get reference position, velocity, effort from the planner
  std::string name[joint_size_];
  double pos_ref[joint_size_];
  double vel_ref[joint_size_];
  double eff_ref[joint_size_];
  ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory joint_cmd_buffer = *(joint_cmd_buffer_.readFromRT());
  for (size_t i = 0; i < joint_size_; i++) {
    name[i] = joint_cmd_buffer.name[i];
    pos_ref[i] = joint_cmd_buffer.positions[i];
    vel_ref[i] = joint_cmd_buffer.velocities[i];
    eff_ref[i] = joint_cmd_buffer.efforts[i];
  }

  // TODO(Jaehyun): Interpolate received reference data if needed
  // Check the email sent from Vlad

  // Compute effort command
  double eff_cmd[joint_size_];
  for (size_t i = 0; i < joint_size_; i++) {
    eff_cmd[i] = eff_ref[i] + kp_[i] * (vel_ref[i]- vel_prev[i]) + kd_[i] * (pos_ref[i] - pos_prev[i]);
  }

  // Save previous position and velocity
  for (size_t i = 0; i < joint_size_; i++) {
    pos_prev[i] = pos_curr[i];
    vel_prev[i] = vel_curr[i];
  }

  // Send effort command to motors
  for (size_t i = 0; i < joint_size_; i++) {
    joint_handle_[i].setCommand(eff_cmd[i]);
  }

  // Publish joint_state data
  if (rt_joint_state_pub_->trylock()) {
    rt_joint_state_pub_->msg.header.stamp = curr_time;
    rt_joint_state_pub_->msg.name = joint_name_;
    rt_joint_state_pub_->msg.position = pos_curr;
    rt_joint_state_pub_->msg.velocity = vel_curr;
    rt_joint_state_pub_->unlockAndPublish();
  }

  // Publish imu data
  if (rt_imu_pub_->trylock()) {
    rt_imu_pub_->msg_.header.stamp = curr_time;
    rt_imu_pub_->msg_.header.frame_id = imu_handle_[i].getFrameId();
    rt_imu_pub_->msg_.angular_velocity = ang_vel;
    rt_imu_pub_->msg_.linear_acceleration = lin_acc;
    rt_imu_pub_->unlockAndPublish();
  }
}

void SoloController::stopping(const ros::Time& /*time*/) {}

} // namespace solo_controller
PLUGINLIB_EXPORT_CLASS(solo_controller::SoloController, controller_interface::ControllerBase)
