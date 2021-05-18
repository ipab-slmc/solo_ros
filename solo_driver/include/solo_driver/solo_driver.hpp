// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#ifndef SOLO_DRIVER__SOLO_DRIVER_HPP_
#define SOLO_DRIVER__SOLO_DRIVER_HPP_

// #include <robot_interfaces/robot_frontend.hpp>
// #include <robot_interfaces/sensors/sensor_frontend.hpp>
#include <ros/ros.h>

#include <memory>
#include <vector>

// Reference: https://github.com/open-dynamic-robot-initiative/solo/blob/master/src/solo12.cpp

namespace solo_driver
{
// Reference: https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/types.hpp
// class Action
// {
// public:
//   int values[2];

//   void print(bool backline) const
//   {
//     std::cout << "action: " << values[0] << " " << values[1] << " ";
//     if (backline) {std::cout << "\n";}
//   }

//   template<class Archive>
//   void serialize(Archive & ar)
//   {
//     ar(values);
//   }
// };

// Reference: https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/types.hpp
// class Observation
// {
// public:
//   int values[2];

//   void print(bool backline) const
//   {
//     std::cout << "observation: " << values[0] << " " << values[1] << " ";
//     if (backline) {std::cout << "\n";}
//   }

//   template<class Archive>
//   void serialize(Archive & ar)
//   {
//     ar(values);
//   }
// };

class SoloDriver
{
public:
  SoloDriver();
  ~SoloDriver() {}

  void init(uint8_t joint_size)
  {
    joint_size_ = joint_size;
    joint_data_.resize(joint_size_);
  }

  // Joint
  // TODO(JaehyunShim): Need more consideration on public read and write functions
  std::vector<double> read_joint();
  bool write_joint(std::vector<double> cmd);

  // IMU
  double read_sensor();

private:
  uint8_t joint_size_;
  std::vector<double> joint_data_;

  // Reference for robot_frontend
  // https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/demo_multiprocess_frontend.cpp
  // std::shared_ptr<robot_interfaces::MultiProcessRobotData<Action, Observation>> robot_data_ptr_;
  // robot_interfaces::RobotFrontend<Action, Observation> robot_frontend_;
  // Action action_;
  // Observation robot_observation_;

  // Reference for sensor_frontend
  // https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/sensors/sensor_frontend.hpp
  // std::shared_ptr<robot_interfaces::SensorData<ObservationType>> sensor_data_ptr_;
  // robot_interfaces::SensorFrontend<ObservationType> sensor_frontend_;
  // Observation sensor_observation_;

  // robot_interfaces::TimeIndex t_index_;
};

}  // namespace solo_driver
#endif  // SOLO_DRIVER__SOLO_DRIVER_HPP_
