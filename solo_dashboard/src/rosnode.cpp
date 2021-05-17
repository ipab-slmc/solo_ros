// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>

#include "solo_dashboard/rosnode.hpp"

namespace solo_dashboard
{
RosNode::RosNode()
: nh_(""),
  private_nh_("~")
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize ROS publishers and subscribers
  // TODO(JaehyunShim): more consideration on queue size
  chatter_pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
  chatter_sub_ = nh_.subscribe("chatter", 10, &RosNode::chatter_callback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  solo_test_pub_ = nh_.advertise<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>(
    "joint_cmd", 10);

  // Initialize ROS timer
  timer_ = nh_.createTimer(ros::Duration(0.01), &RosNode::timer_callback, this);
}

void RosNode::timer_callback(const ros::TimerEvent & te)
{
  static int count = 0;
  if (pub_onoff_ == true) {
    std::stringstream ss;
    ss << "hello world " << count;
    auto msg = std_msgs::String();
    msg.data = ss.str();
    chatter_pub_.publish(msg);
    ROS_INFO("Publisher: %s", msg.data.c_str());
    count++;
  }

  auto cmd_vel_msg = geometry_msgs::Twist();
  cmd_vel_msg.linear.x = lin_vel_;
  cmd_vel_msg.angular.z = ang_vel_;
  cmd_vel_pub_.publish(cmd_vel_msg);

  // auto joint_cmd_msg = ipab_controller_msgs::EffortFeedforwardWithJointFeedback();
  // solo_test_pub_.publish(joint_cmd_msg);
}

void RosNode::chatter_callback(const std_msgs::String::ConstPtr & msg)
{
  if (sub_onoff_ == true) {
    ROS_INFO("Subscriber: %s", msg->data.c_str());
  }
}
}  // namespace solo_dashboard
