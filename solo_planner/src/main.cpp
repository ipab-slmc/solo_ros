// TODO(JaehyunShim): Write copyright
//
// Copyright (c) 2021, University of Edinburgh
//
//
// Check what license will be used.
//
//

#include "solo_planner/solo_planner.hpp"

int main(int argc, char * argv[])
{
  // Init
  ros::init(argc, argv, "solo_planner");
  ros::NodeHandle nh("");
  solo_planner::SoloPlanner solo_planner;

  // Update
  ros::Rate r(1000);  // 1000[Hz]
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  // Shutdown
  ros::shutdown();
  return 0;
}
