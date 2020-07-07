/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "sample_sub/sample_sub.h"

void msgCallback(const std_msgs::Header::ConstPtr& msg) 
{ 
  ROS_INFO("received data: %d", msg->seq); 
}

/**
 * Main function
 * @param argc Number of argument
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_sub");
  ros::NodeHandle nh;
  ros::Subscriber ros_sub;
  ros_sub = nh.subscribe<std_msgs::Header>("ros_cfe_msg", 100, msgCallback);
  ros::spin();
  return 0;
}
