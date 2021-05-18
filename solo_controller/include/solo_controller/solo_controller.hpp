// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedback.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <solo_controller/SoloControllerConfig.h>
#include <urdf/model.h>

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

  // ROS Publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> rt_joint_state_pub_;

  // ROS Subscriber
  realtime_tools::RealtimeBuffer<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>
  joint_cmd_buffer_;
  ros::Subscriber joint_cmd_sub_;
  void joint_cmd_callback(
    const ipab_controller_msgs::EffortFeedforwardWithJointFeedback::ConstPtr & msg)
  {
    joint_cmd_buffer_.writeFromNonRT(*msg);
  }

  // ROS Dynamic Reconfigure Server
  std::vector<std::shared_ptr<dynamic_reconfigure::Server<solo_controller::SoloControllerConfig>>>
  dyn_reconf_server_;
  boost::recursive_mutex dyn_reconf_mutex_;
  std::vector<realtime_tools::RealtimeBuffer<solo_controller::SoloControllerConfig>>
  solo_controller_config_;
  void dyn_reconf_callback(
    solo_controller::SoloControllerConfig & solo_controller_config,
    uint32_t /*level*/, uint8_t index)
  {
    solo_controller_config_[index].writeFromNonRT(solo_controller_config);
  }

  // Joint limit enforcement
  std::vector<urdf::JointConstSharedPtr> joint_urdf_;
  void enforce_joint_limit(double & cmd, uint8_t index)
  {
    if (joint_urdf_[index]->type == urdf::Joint::REVOLUTE ||
      joint_urdf_[index]->type == urdf::Joint::PRISMATIC)
    {
      if (std::abs(cmd) > joint_urdf_[index]->limits->effort) {
        cmd = joint_urdf_[index]->limits->effort * (cmd / std::abs(cmd));
      }
    }
  }
};
}  // namespace solo_controller
#endif  // SOLO_CONTROLLER__SOLO_CONTROLLER_HPP_
