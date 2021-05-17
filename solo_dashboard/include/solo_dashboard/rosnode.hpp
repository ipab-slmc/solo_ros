// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_DASHBOARD__ROSNODE_HPP_
#define SOLO_DASHBOARD__ROSNODE_HPP_

#include <QStringListModel>

#include <geometry_msgs/Twist.h>
#include <ipab_controller_msgs/EffortFeedforwardWithJointFeedback.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

namespace solo_dashboard
{
class RosNode
{
public:
  RosNode();
  ~RosNode() {}

  bool pub_onoff_ = false;
  bool sub_onoff_ = false;
  bool solo_test_onoff_ = true;
  double lin_vel_ = 0.0;
  double ang_vel_ = 0.0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher chatter_pub_;
  ros::Subscriber chatter_sub_;
  void chatter_callback(const std_msgs::String::ConstPtr & msg);
  ros::Publisher cmd_vel_pub_;
  ros::Publisher solo_test_pub_;
  ros::Timer timer_;
  void timer_callback(const ros::TimerEvent & te);
};
}  // namespace solo_dashboard
#endif  // SOLO_DASHBOARD__ROSNODE_HPP_
