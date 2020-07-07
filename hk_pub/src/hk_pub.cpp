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
#include "hk_pub/hk_pub.h"
#include <sys/time.h>
#include <time.h>

/**
 * Main function
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Result
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "hk_pub");
  ros::NodeHandle nh;
  ros::Publisher ros_pub;
  ros_pub = nh.advertise<relay_node::CFE_SB_CmdHdr>("/cfs_hk/ground_command", 100);
  ros::Rate loop_rate(100);
  int count = 0;
  while (ros::ok()) {
    printf("please enter key.\n");
    getchar();
    relay_node::CFE_SB_CmdHdr msg;

    msg.Pri.StreamId[0] = 0;
    msg.Pri.StreamId[1] = 0;
    msg.Pri.Sequence[0] = 0;
    msg.Pri.Sequence[1] = 0;
    msg.Pri.Length[0] = 0;
    msg.Pri.Length[1] = 0;
    msg.Sec.Command = 0;
    ros_pub.publish(msg);
    // ROS_INFO("[relay_node] published /cfs_hk/ground_command.");
    struct timeval myTime;
    struct tm *time_st;
    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);
    printf("[command_pub][%02d:%02d.%06d] publish /cfs_hk/ground_command\n",
           time_st->tm_min, time_st->tm_sec, myTime.tv_usec);
    loop_rate.sleep();
    count++;
  }
  return 0;
}
