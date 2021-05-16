// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "solo_dashboard/rosnode.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace solo_dashboard
{
RosNode::RosNode()
: Node("solo_dashboard")
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // ROS Publisher & Subscriber & Client
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  chatter_pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);
  chatter_sub_ =
    this->create_subscription<std_msgs::msg::String>(
    "chatter",
    qos,
    std::bind(&RosNode::chatter_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // ROS Timer
  timer_ = this->create_wall_timer(
    10ms,
    std::bind(&RosNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Initialized rqt example node");
}

RosNode::~RosNode()
{
  RCLCPP_INFO(this->get_logger(), "Terminated rqt example node");
}

void RosNode::timer_callback()
{
  static int count = 0;
  if (pub_onoff_ == true) {
    std::stringstream ss;
    ss << "hello world " << count;
    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    chatter_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publisher: %s", msg.data.c_str());
    count++;
  }

  auto msg2 = geometry_msgs::msg::Twist();
  msg2.linear.x = lin_vel_;
  msg2.angular.z = ang_vel_;
  cmd_vel_pub_->publish(msg2);
}

void RosNode::chatter_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (sub_onoff_ == true) {
    RCLCPP_INFO(this->get_logger(), "Subscriber: %s", msg->data.c_str());
  }
}
}  // namespace solo_dashboard
