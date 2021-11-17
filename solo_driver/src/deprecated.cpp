// Deprecated
// To be removed

//
//
//
//
//
//
//
//
//
//
//
//
//
//
// Copyright (c) 2021, University of Edinburgh
//
// #include <robot_interfaces/robot_frontend.hpp>
// #include <robot_interfaces/sensors/sensor_frontend.hpp>

// namespace solo_driver
// {
// class SoloDriver
// {
//   Reference for robot_frontend
//   https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/demo_multiprocess_frontend.cpp
//   std::shared_ptr<robot_interfaces::MultiProcessRobotData<Action, Observation>> robot_data_ptr_;
//   robot_interfaces::RobotFrontend<Action, Observation> robot_frontend_;
//   Action action_;
//   Observation robot_observation_;

//   Reference for sensor_frontend
//   https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/include/robot_interfaces/sensors/sensor_frontend.hpp
//   std::shared_ptr<robot_interfaces::SensorData<ObservationType>> sensor_data_ptr_;
//   robot_interfaces::SensorFrontend<ObservationType> sensor_frontend_;
//   Observation sensor_observation_;

//   robot_interfaces::TimeIndex t_index_;
// }
// }  // namespace solo_driver

// SoloDriver::solo_driver()
// {
//   // Init robot frontend
//   robot_data_ptr_ =
//     std::make_shared<robot_interfaces::MultiProcessRobotData<Action, Observation>>(
//       "robot_data", false);
//   robot_frontend_ = robot_interfaces::RobotFrontend<Action, Observation>(robot_data_ptr_);

//   Init sensor frontend
//   sensor_data_ptr_ = std::make_shared<robot_interfaces::SensorData<ObservationType>>(
//     "sensor_data", false);
//   sensor_frontend_ = robot_interfaces::SensorFrontend<ObservationType>(sensor_data_ptr_);
// }

// std::vector<double> SoloDriver::read_joint()
// {
//   // TODO(JaehyunShim): Write read_onoff switch code
//   // Write has to be done first
//   // or maybe t_index_ is not really needed..?
//   // Should it be read -> process -> write loop or write -> process -> read?

//   // Get robot observation
//   // robot_observation_ = robot_frontend_.get_observation(t_index_);
//   // robot_observation_.print(true);
//   // static double robot_data[joint_size_];
//   // for (size_t i = 0; i < joint_size_; i++) {
//   //   robot_data[i] = robot_observation_.values[i];
//   // }

//   // TODO(JaehyunShim): Write read_onoff switch code

//   // Get sensor observation
//   // sensor_observation_ = sensor_frontend_.get_observation(t_index_);
//   // sensor_observation_.print(true);
//   // sensor_data = sensor_observation_.values[0];

// }

// void SoloDriver::write_joint(std::vector<double> cmd)
// {
//   Set robot action
//   t_index_ = frontend.append_desired_action(action);
//   action.print(false);  // set to true if you want to print action
// }

// Reference:
// https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/types.hpp
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

// Reference:
// https://github.com/open-dynamic-robot-initiative/robot_interfaces/blob/master/demos/types.hpp
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
