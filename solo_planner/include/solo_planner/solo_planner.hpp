// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedback.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <string>
#include <vector>

#ifndef SOLO_PLANNER__SOLO_PLANNER_HPP_
#define SOLO_PLANNER__SOLO_PLANNER_HPP_

namespace solo_planner
{
class SoloPlanner
{
public:
  SoloPlanner();
  ~SoloPlanner() {}

private:
  // Node Handle
  ros::NodeHandle nh_;

  // Joint
  uint8_t joint_size_;
  std::vector<std::string> joint_name_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> pos_curr_;
  std::vector<double> vel_curr_;
  std::vector<double> pos_ref_;
  std::vector<double> vel_ref_;
  std::vector<double> eff_ref_;

  // IMU
  sensor_msgs::Imu imu_;

  // ROS Publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<
      ipab_controller_msgs::EffortFeedforwardWithJointFeedback>> rt_joint_cmd_pub_;

  // ROS Subscriber
  realtime_tools::RealtimeBuffer<sensor_msgs::JointState> joint_state_buffer_;
  ros::Subscriber joint_state_sub_;
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr & msg)
  {
    joint_state_buffer_.writeFromNonRT(*msg);
  }

  realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_buffer_;
  ros::Subscriber imu_sub_;
  void imu_callback(const sensor_msgs::Imu::ConstPtr & msg)
  {
    imu_buffer_.writeFromNonRT(*msg);
  }

  // ROS Timer
  ros::Timer timer_;
  void timer_callback(const ros::TimerEvent & te);
};
}  // namespace solo_planner
#endif  // SOLO_PLANNER__SOLO_PLANNER_HPP_
