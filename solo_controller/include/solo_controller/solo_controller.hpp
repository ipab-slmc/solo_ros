// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <controller_interface/controller.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedbackTrajectory.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <string>
#include <vector>

#ifndef SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_
#define SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_

namespace solo_controller
{
class SoloController
: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  SoloController() {}
  ~SoloController() {}

  bool init(
    hardware_interface::ImuSensorInterface * hw,
    ros::NodeHandle & root_nh,
    ros::NodeHandle & controller_nh);
  void starting(const ros::Time & time);
  void update(const ros::Time & time, const ros::Duration & period);
  void stopping(const ros::Time & time);

private:
  // Update on/off switch for the first loop
  bool update_onoff = false;

  // Joint
  uint8_t joint_size_;
  std::vector<std::string> joint_name_;
  std::vector<hardware_interface::JointHandle> joint_handle_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double pos_curr_;
  double vel_curr_;
  double pos_prev_;
  double vel_prev_;
  double pos_ref_;
  double vel_ref_;
  double eff_ref_;
  double eff_cmd_;

  // IMU
  std::string imu_name_;
  hardware_interface::ImuSensorHandle imu_handle_;

  // ROS Publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> rt_joint_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> rt_imu_pub_;

  // ROS Subscribers
  realtime_tools::RealtimeBuffer<ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory>
  joint_cmd_buffer_;
  ros::Subscriber joint_cmd_sub_;
  void joint_cmd_callback(
    const ipab_controller_msgs::EffortFeedforwardWithJointFeedbackTrajectory & msg)
  {
    joint_cmd_buffer_.writeFromNonRT(msg);
  }
};
}  // namespace solo_controller
#endif  // SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_
