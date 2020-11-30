/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

#include "test_pub/test_pub.h"

/**
 * Main function
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub");
  ros::NodeHandle nh;
  ros::Publisher ros_pub;
  ros_pub = nh.advertise<geometry_msgs::Pose>("ros_cfe_msg2", 100);
  ros::Rate loop_rate(100);
  int count = 0;
  while (ros::ok()) {
    geometry_msgs::Pose msg;
    msg.position.x = (double)count;
    ROS_INFO("send msg.position.x: %lf, rostimenow.toSec: %f", msg.position.x, ros::Time::now().toSec());
    ros_pub.publish(msg);
    loop_rate.sleep();
    count++;
  }
  return 0;
}
