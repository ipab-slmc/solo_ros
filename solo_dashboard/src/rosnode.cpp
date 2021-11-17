// Copyright 2021 University of Edinburgh
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>
#include <sstream>
#include <string>

#include "solo_dashboard/rosnode.hpp"

namespace solo_dashboard
{
Rosnode::Rosnode() : nh_(""), private_nh_("~")
{
  // Initialize ROS publishers and subscribers
  // TODO(JaehyunShim): Need more consideration on queue size
  chatter_pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
  chatter_sub_ = nh_.subscribe("chatter", 10, &Rosnode::chatter_callback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  solo_test_pub_ =
      nh_.advertise<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>("joint_command", 10);

  // Initialize ROS timer
  timer_ = nh_.createTimer(ros::Duration(0.01), &Rosnode::timer_callback, this);
}

void Rosnode::timer_callback(const ros::TimerEvent & te)
{
  static int count = 0;
  if (pub_onoff_ == true)
  {
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

  // auto joint_command_msg = ipab_controller_msgs::EffortFeedforwardWithJointFeedback();
  // solo_test_pub_.publish(joint_command_msg);
}

void Rosnode::chatter_callback(const std_msgs::String::ConstPtr & msg)
{
  if (sub_onoff_ == true)
  {
    ROS_INFO("Subscriber: %s", msg->data.c_str());
  }
}
}  // namespace solo_dashboard
