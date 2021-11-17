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
  ~SoloPlanner()
  {
  }

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
  std::shared_ptr<
      realtime_tools::RealtimePublisher<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>>
      rt_joint_command_pub_;

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
