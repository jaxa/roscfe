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
#include "ros/ros.h"

// base
#include "relay_node/communication/communication.h"
#include "convert/subscriber.h"

/**
 * Main function
 * @param argc Number of arguments
 * @param argv Arguments
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "relay_node");

  ros::NodeHandle nh;

  try {
    // wake up connection
    Communication comm(nh);

    comm.init();
    ROS_INFO("Connect is success. ");

    // set publisher
    comm.advertise();

    // set subscriber
    Subscriber sub(nh, &comm);
    sub.subscribe();

    ros::Rate loop(1000);

    while(ros::ok()) {
      ros::spinOnce();
      comm.publish_recv_data();
      loop.sleep();
    }
  } catch(const std::runtime_error &ex) {
    ROS_ERROR("%s", ex.what());
  }
}

