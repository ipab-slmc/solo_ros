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
class Rosnode
{
public:
  Rosnode();
  ~Rosnode()
  {
  }

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
