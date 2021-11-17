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

#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <memory>

#include "solo_controller/solo_controller.hpp"

namespace solo_controller
{
bool SoloController::init(hardware_interface::EffortJointInterface * hw, ros::NodeHandle & root_nh,
                          ros::NodeHandle & controller_nh)
{
  // TODO(JaehyunShim): Add exception throwing lines
  controller_nh.getParam("joint", joint_name_);
  joint_size_ = joint_name_.size();

  // Resize joint data vectors
  for (size_t i = 0; i < joint_size_; i++)
  {
    kp_.emplace_back(0.0);
    kd_.emplace_back(0.0);
    pos_curr_.emplace_back(0.0);
    vel_curr_.emplace_back(0.0);
    pos_prev_.emplace_back(0.0);
    vel_prev_.emplace_back(0.0);
    pos_ref_.emplace_back(0.0);
    vel_ref_.emplace_back(0.0);
    eff_ref_.emplace_back(0.0);
    eff_cmd_.emplace_back(0.0);
  }

  // Get PD gains from parameter server
  for (size_t i = 0; i < joint_size_; i++)
  {
    controller_nh.getParam("gain/" + joint_name_[i] + "/pid/p", kp_[i]);
    controller_nh.getParam("gain/" + joint_name_[i] + "/pid/d", kd_[i]);
  }

  // Initialize whole body state
  // TODO(JaehyunShim)
  // wb_state_.centroidal;
  wb_state_.joints.resize(joint_size_);
  for (size_t i = 0; i < joint_size_; i++)
  {
    wb_state_.joints[i].name = joint_name_[i];
    wb_state_.joints[i].position = 0.0;
    wb_state_.joints[i].velocity = 0.0;
    wb_state_.joints[i].effort = 0.0;
  }
  // wb_state_.contacts;

  // Joint Handle
  for (size_t i = 0; i < joint_size_; i++)
  {
    try
    {
      joint_handle_.emplace_back(hw->getHandle(joint_name_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException & e)
    {
      ROS_ERROR_STREAM("Error: " << e.what());
      return false;
    }
  }

  // Initialize ROS publishers
  // TODO(JaehyunShim): Need more consideration on the queue size
  rt_joint_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
      controller_nh, "joint_states", 10));

  // TODO(JaehyunShim): Need more consideration on the queue size
  // rt_tf_pub_.reset(
  //   new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(
  //     root_nh, "tf", 10));

  // TODO(JaehyunShim): Need more consideration on the queue size
  planner_start_pub = controller_nh.advertise<std_msgs::Empty>("planner_start", 1);

  rt_wb_state_pub_.reset(
      new realtime_tools::RealtimePublisher<whole_body_state_msgs::WholeBodyState>(
          controller_nh, "whole_body_state", 10));

  rt_wb_traj_pub_.reset(
      new realtime_tools::RealtimePublisher<whole_body_state_msgs::WholeBodyTrajectory>(
          controller_nh, "whole_body_trajectory", 10));

  // Initialize ROS subscribers
  // TODO(JaehyunShim): Need more consideration on the queue size
  joint_command_sub_ =
      controller_nh.subscribe<ipab_controller_msgs::EffortFeedforwardWithJointFeedback>(
          "joint_command", 10, &SoloController::joint_command_callback, this);

  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_command_buffer;
  joint_command_buffer.positions.resize(joint_size_);
  joint_command_buffer.velocities.resize(joint_size_);
  joint_command_buffer.efforts.resize(joint_size_);
  joint_command_buffer.position_gains.resize(joint_size_);
  joint_command_buffer.velocity_gains.resize(joint_size_);
  for (size_t i = 0; i < joint_size_; i++)
  {
    joint_command_buffer.positions[i] = 0.0;
    joint_command_buffer.velocities[i] = 0.0;
    joint_command_buffer.efforts[i] = 0.0;
    joint_command_buffer.position_gains[i] = 0.0;
    joint_command_buffer.velocity_gains[i] = 0.0;
  }
  joint_command_buffer_.writeFromNonRT(joint_command_buffer);

  // Dynamic reconfigure for PD gains
  // Reference: https://github.com/ros-controls/control_toolbox/blob/melodic-devel/src/pid.cpp
  for (size_t i = 0; i < joint_size_; i++)
  {
    std::string dyn_reconf_nh_ns =
        controller_nh.getNamespace() + "/gain/" + joint_name_[i] + "/pid";
    ros::NodeHandle dyn_reconf_nh(dyn_reconf_nh_ns);
    dyn_reconf_server_.emplace_back(
        std::make_shared<dynamic_reconfigure::Server<solo_controller::SoloControllerConfig>>(
            dyn_reconf_mutex_, dyn_reconf_nh));
  }
  solo_controller_config_.resize(joint_size_);

  solo_controller::SoloControllerConfig solo_controller_config;
  for (size_t i = 0; i < joint_size_; i++)
  {
    // Set init param values
    solo_controller_config.p = kp_[i];
    solo_controller_config.d = kd_[i];
    dyn_reconf_mutex_.lock();
    dyn_reconf_server_[i]->updateConfig(solo_controller_config);
    dyn_reconf_mutex_.unlock();

    // Set server callback
    dyn_reconf_server_[i]->setCallback(
        boost::bind(&SoloController::dyn_reconf_callback, this, _1, _2, i));
  }

  // URDF parsing for enforcing joint limit
  urdf::Model urdf;
  ros::NodeHandle urdf_nh("");
  if (!urdf.initParamWithNodeHandle("robot_description", urdf_nh))
  {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  // TODO(JaehyunShim): Add exception throwing lines
  for (size_t i = 0; i < joint_size_; i++)
  {
    joint_urdf_.emplace_back(urdf.getJoint(joint_name_[i]));
  }

  return true;
}

void SoloController::starting(const ros::Time & /*time*/)
{
}

void SoloController::update(const ros::Time & time, const ros::Duration & period)
{
  // Get current time
  ros::Time curr_time = time;
  // TODO(Jaehyun): Check why sometimes time becomes negative
  // ROS_INFO("Current Time: %d", time.toNSec());

  // Get joint data
  for (size_t i = 0; i < joint_size_; i++)
  {
    pos_curr_[i] = joint_handle_[i].getPosition();
    vel_curr_[i] = joint_handle_[i].getVelocity();
  }

  // TODO(Jaehyun): Rewrite this chunk of code if there is a more
  // clean and intuitive way to process the first loop.
  // First loop for getting previous position and velocity
  if (!update_onoff)
  {
    for (size_t i = 0; i < joint_size_; i++)
    {
      pos_prev_[i] = pos_curr_[i];
      vel_prev_[i] = vel_curr_[i];
    }
    update_onoff = true;

    // Planner Start
    std_msgs::Empty msg;
    planner_start_pub.publish(msg);

    return;
  }

  // Get reference position, velocity, effort from the planner
  ipab_controller_msgs::EffortFeedforwardWithJointFeedback joint_command_buffer =
      *(joint_command_buffer_.readFromRT());
  for (size_t i = 0; i < joint_size_; i++)
  {
    // TODO(Jaehyun): Add lines checking if joint_name equals joint_command_buffer.name[i]
    pos_ref_[i] = joint_command_buffer.positions[i];
    vel_ref_[i] = joint_command_buffer.velocities[i];
    eff_ref_[i] = joint_command_buffer.efforts[i];
    kp_[i] = joint_command_buffer.position_gains[i];
    kd_[i] = joint_command_buffer.velocity_gains[i];
  }

  // TODO(Jaehyun): Interpolate received reference data if needed

  // Compute effort command
  for (size_t i = 0; i < joint_size_; i++)
  {
    eff_cmd_[i] =
        eff_ref_[i] + kp_[i] * (pos_ref_[i] - pos_prev_[i]) + kd_[i] * (vel_ref_[i] - vel_prev_[i]);
  }

  // Save previous position and velocity
  for (size_t i = 0; i < joint_size_; i++)
  {
    pos_prev_[i] = pos_curr_[i];
    vel_prev_[i] = vel_curr_[i];
  }

  // Send effort command to motors
  for (size_t i = 0; i < joint_size_; i++)
  {
    enforce_joint_limit(eff_cmd_[i], i);
    joint_handle_[i].setCommand(eff_cmd_[i]);
    ROS_INFO("%d joint command: %lf", i, eff_cmd_[i]);
  }

  // Publish joint_state data
  if (rt_joint_state_pub_->trylock())
  {
    rt_joint_state_pub_->msg_.header.stamp = curr_time;
    rt_joint_state_pub_->msg_.name = joint_name_;
    rt_joint_state_pub_->msg_.position = pos_curr_;
    rt_joint_state_pub_->msg_.velocity = vel_curr_;
    rt_joint_state_pub_->unlockAndPublish();
  }

  // Update parameters
  // for (size_t i = 0; i < joint_size_; i++) {
  //   solo_controller::SoloControllerConfig solo_controller_config =
  //     *(solo_controller_config_[i].readFromRT());
  //   kp_[i] = solo_controller_config.p;
  //   kd_[i] = solo_controller_config.d;
  //   // ROS_INFO("%d joint kp_: %lf", i, kp_[i]);
  //   // ROS_INFO("%d joint kd_: %lf", i, kd_[i]);
  // }

  // Publish tf
  // TODO(Jaehyun): Do proper TF calc
  // Bug: https://github.com/ros/geometry2/issues/467 from OR
  // geometry_msgs::TransformStamped temp_tf;
  // temp_tf.header.stamp = curr_time;
  // temp_tf.header.frame_id = "world";
  // temp_tf.child_frame_id = "base_link";
  // temp_tf.transform.translation.x = 0.0;
  // temp_tf.transform.translation.y = 0.0;
  // temp_tf.transform.translation.z = 0.5;
  // temp_tf.transform.rotation.x = 0.0;
  // temp_tf.transform.rotation.y = 0.0;
  // temp_tf.transform.rotation.z = 0.0;
  // temp_tf.transform.rotation.w = 1.0;
  // if (rt_tf_pub_->trylock()) {
  //   rt_tf_pub_->msg_.transforms.emplace_back(temp_tf);
  //   rt_tf_pub_->unlockAndPublish();
  // }

  // Publish whole body state and trajectory
  // TODO(Jaehyun): Will need to think about which node will publish this data
  // Let's publish dummy data for now to see the magic.
  // wb_state_.joints =
  if (rt_wb_state_pub_->trylock())
  {
    rt_wb_state_pub_->msg_.header.stamp = curr_time;
    rt_wb_state_pub_->msg_.header.frame_id = "base_link";
    rt_wb_state_pub_->msg_.time = curr_time.toSec();  // TODO(Jaehyun): Redundant, Already in header
    // rt_wb_state_pub_->msg_.centroidal = ;
    for (size_t i = 0; i < joint_size_; i++)
    {
      rt_wb_state_pub_->msg_ = wb_state_;
    }
    // rt_wb_state_pub_->msg_.contacts = ;
    rt_wb_state_pub_->unlockAndPublish();
  }

  // Dummy
  wb_state_traj_.emplace_back(wb_state_);
  wb_state_traj_.emplace_back(wb_state_);
  wb_state_traj_.emplace_back(wb_state_);
  wb_state_traj_.emplace_back(wb_state_);
  if (rt_wb_traj_pub_->trylock())
  {
    for (size_t i = 0; i < joint_size_; i++)
    {
      rt_wb_traj_pub_->msg_.actual = wb_state_;
      rt_wb_traj_pub_->msg_.trajectory = wb_state_traj_;
    }
    rt_wb_traj_pub_->unlockAndPublish();
  }
}

void SoloController::stopping(const ros::Time & /*time*/)
{
}

}  // namespace solo_controller

PLUGINLIB_EXPORT_CLASS(solo_controller::SoloController, controller_interface::ControllerBase)
