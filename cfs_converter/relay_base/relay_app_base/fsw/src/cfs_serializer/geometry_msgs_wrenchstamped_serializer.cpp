/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_wrenchstamped_serializer.h"

int serialize_geometry_msgs_wrenchstamped(const ::geometry_msgs::WrenchStamped &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_std_msgs_header(send_data.header, buffer + len);

  len += serialize_geometry_msgs_wrench(send_data.wrench, buffer + len);

  return len;
}

geometry_msgs::WrenchStamped deserialize_geometry_msgs_wrenchstamped(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::WrenchStamped msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.header = deserialize_std_msgs_header(buffer + length, len);
  length += len;

  msg.wrench = deserialize_geometry_msgs_wrench(buffer + length, len);
  length += len;

  return msg;
}


