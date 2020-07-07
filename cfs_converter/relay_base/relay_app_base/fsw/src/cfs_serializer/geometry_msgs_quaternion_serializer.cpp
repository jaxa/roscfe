/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_quaternion_serializer.h"

int serialize_geometry_msgs_quaternion(const ::geometry_msgs::Quaternion &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_float(send_data.x, buffer + len);
  len += serialize_float(send_data.y, buffer + len);
  len += serialize_float(send_data.z, buffer + len);
  len += serialize_float(send_data.w, buffer + len);

  return len;
}

geometry_msgs::Quaternion deserialize_geometry_msgs_quaternion(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::Quaternion msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.x = deserialize_float(buffer + length, len);
  length += len;
  msg.y = deserialize_float(buffer + length, len);
  length += len;
  msg.z = deserialize_float(buffer + length, len);
  length += len;
  msg.w = deserialize_float(buffer + length, len);
  length += len;

  return msg;
}

