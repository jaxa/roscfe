/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_wrench_serializer.h"

int serialize_geometry_msgs_wrench(const ::geometry_msgs::Wrench &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_vector3(send_data.force, buffer + len);

  len += serialize_geometry_msgs_vector3(send_data.torque, buffer + len);

  return len;
}
geometry_msgs::Wrench deserialize_geometry_msgs_wrench(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::Wrench msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.force = deserialize_geometry_msgs_vector3(buffer + length, len);
  length += len;

  msg.torque = deserialize_geometry_msgs_vector3(buffer + length, len);
  length += len;

  return msg;
}
