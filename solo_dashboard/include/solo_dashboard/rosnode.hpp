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
#include <ros/ros.h>
#include <std_msgs/msg/String.h>

#include <string>

namespace solo_dashboard
{
class RosNode : public rclcpp::Node
{
public:
  RosNode();
  virtual ~RosNode();

  bool pub_onoff_ = true;
  bool sub_onoff_ = false;
  double lin_vel_ = 0.0;
  double ang_vel_ = 0.0;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void chatter_callback(const std_msgs::msg::String::SharedPtr msg);
  void timer_callback();
};
}  // namespace solo_dashboard
#endif  // SOLO_DASHBOARD__ROSNODE_HPP_
