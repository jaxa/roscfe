/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/nav_msgs_odometry_serializer.h"

#include "../convert/nav_msgs/Odometry.h"

int serialize_nav_msgs_odometry(const ::nav_msgs::Odometry &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_std_msgs_header(send_data.header, buffer + len);

  len += serialize_geometry_msgs_posewithcovariance(send_data.pose, buffer + len);

  len += serialize_geometry_msgs_twistwithcovariance(send_data.twist, buffer + len);

  return len;
}

nav_msgs::Odometry deserialize_nav_msgs_odometry(const unsigned char *buffer, unsigned int &length) {
  nav_msgs::Odometry msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.header = deserialize_std_msgs_header(buffer + length, len);
  length += len;

  msg.pose = deserialize_geometry_msgs_posewithcovariance(buffer + length, len);
  length += len;

  msg.twist = deserialize_geometry_msgs_twistwithcovariance(buffer + length, len);
  length += len;

  return msg;
}

