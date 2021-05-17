// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedback.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <string>
#include <vector>

#ifndef SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_
#define SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_

namespace solo_controller
{
class SoloController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  SoloController() {}
  ~SoloController() {}

  bool init(
    hardware_interface::EffortJointInterface * robot_hw,
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
  std::vector<double> pos_curr_;
  std::vector<double> vel_curr_;
  std::vector<double> pos_prev_;
  std::vector<double> vel_prev_;
  std::vector<double> pos_ref_;
  std::vector<double> vel_ref_;
  std::vector<double> eff_ref_;
  std::vector<double> eff_cmd_;

  // ROS Publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> rt_joint_state_pub_;

  // ROS Subscribers
  realtime_tools::RealtimeBuffer<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>
  joint_cmd_buffer_;
  ros::Subscriber joint_cmd_sub_;
  void joint_cmd_callback(
    const ipab_controller_msgs::EffortFeedforwardWithJointFeedback::ConstPtr & msg)
  {
    joint_cmd_buffer_.writeFromNonRT(*msg);  // TODO(JaehyunShim): Check if the type of the input *msg is correct
  }
};
}  // namespace solo_controller
#endif  // SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_
