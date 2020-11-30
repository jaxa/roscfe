/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "test_sub/test_sub.h"

void msgCallback(const geometry_msgs::Pose::ConstPtr& msg) 
{ 
  ROS_INFO("received data: %lf", msg->position.x); 
}

/**
 * Main function
 * @param argc Number of argument
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_sub");
  ros::NodeHandle nh;
  ros::Subscriber ros_sub;
  ros_sub = nh.subscribe<geometry_msgs::Pose>("ros_cfe_msg2", 100, msgCallback);
  ros::spin();
  return 0;
}
