// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include <solo_driver/solo_driver.hpp>

#include <memory>

namespace solo_driver
{
SoloDriver::SoloDriver()
{
  // Init robot frontend
  robot_data_ptr_ = std::make_shared<robot_interfaces::MultiProcessRobotData<Action, Observation>>(
    "robot_data", false);
  robot_frontend_ = robot_interfaces::RobotFrontend<Action, Observation>(robot_data_ptr_);

  // Init sensor frontend
  sensor_data_ptr_ = std::make_shared<robot_interfaces::SensorData<ObservationType>>(
    "sensor_data", false);
  sensor_frontend_ = robot_interfaces::SensorFrontend<ObservationType>(sensor_data_ptr_);
}

double * SoloDriver::read_joint()
{
  // TODO(JaehyunShim): Write read_onoff switch code
  // Write has to be done first
  // or maybe t_index_ is not really needed..?
  // Should it be read -> process -> write loop or write -> process -> read?

  // Get robot observation
  robot_observation_ = robot_frontend_.get_observation(t_index_);
  robot_observation_.print(true);
  static double robot_data[joint_size_];
  for (size_t i = 0; i < joint_size_; i++) {
    robot_data[i] = robot_observation_.values[i];
  }

  return robot_data;
}

double SoloDriver::read_sensor()
{
  // TODO(JaehyunShim): Write read_onoff switch code

  // Get sensor observation
  sensor_observation_ = sensor_frontend_.get_observation(t_index_);
  sensor_observation_.print(true);
  static double sensor_data;
  sensor_data = sensor_observation_.values[0];

  return sensor_data;
}

bool SoloDriver::write_joint(double * cmd)
{
  // Set robot action
  t_index_ = frontend.append_desired_action(action);
  action.print(false);  // set to true if you want to print action
}
}  // namespace solo_driver
