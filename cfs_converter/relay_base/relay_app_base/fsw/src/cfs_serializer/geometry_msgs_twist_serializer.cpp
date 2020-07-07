/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_twist_serializer.h"

int serialize_geometry_msgs_twist(const geometry_msgs::Twist &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_vector3(send_data.linear, buffer + len);

  len += serialize_geometry_msgs_vector3(send_data.angular, buffer + len);

  return len;
}

geometry_msgs::Twist deserialize_geometry_msgs_twist(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::Twist msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.linear = deserialize_geometry_msgs_vector3(buffer + length, len);
  length += len;

  msg.angular = deserialize_geometry_msgs_vector3(buffer + length, len);
  length += len;

  return msg;
}

