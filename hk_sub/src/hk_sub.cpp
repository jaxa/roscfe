/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

/*************************************************************************
** Includes
*************************************************************************/
#include "hk_sub/hk_sub.h"

#include <sys/time.h>
#include <time.h>

/**
 * Callback function
 * @param msg Received message
 */
void msgCallback(const relay_node::HK_HkPacket::ConstPtr &msg) {
  // ROS_INFO("[hk_sub] CmdCounter: %d, ErrCounter: %d", msg->CmdCounter,
  // msg->ErrCounter);

  struct timeval myTime;
  struct tm *time_st;
  gettimeofday(&myTime, NULL);
  time_st = localtime(&myTime.tv_sec);
  printf("[show_result][%02d:%02d.%06d] subscribe /cfs_hk/hk_telemetry\n",
         time_st->tm_min, time_st->tm_sec, myTime.tv_usec);
  printf("[show_result][%02d:%02d.%06d] CmdCounter = %d \n", time_st->tm_min,
         time_st->tm_sec, myTime.tv_usec, msg->CmdCounter);
}

/**
 * Main function
 * @param argc Number of argument
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "hk_sub");
  ros::NodeHandle nh;
  ros::Subscriber ros_sub;
  ros_sub = nh.subscribe<relay_node::HK_HkPacket>("/cfs_hk/hk_telemetry", 100, msgCallback);
  ros::spin();
  return 0;
}
