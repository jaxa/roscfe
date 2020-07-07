/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

#include "sample_pub/sample_pub.h"

/**
 * Main function
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample_pub");
  ros::NodeHandle nh;
  ros::Publisher ros_pub;
  ros_pub = nh.advertise<std_msgs::Header>("ros_cfe_msg", 100);
  ros::Rate loop_rate(100);
  int count = 0;
  while (ros::ok()) {
    std_msgs::Header msg;
    msg.seq = count;
    ROS_INFO("send msg.data: %d, rostimenow.toSec: %f", msg.seq, ros::Time::now().toSec());
    ros_pub.publish(msg);
    loop_rate.sleep();
    count++;
  }
  return 0;
}
