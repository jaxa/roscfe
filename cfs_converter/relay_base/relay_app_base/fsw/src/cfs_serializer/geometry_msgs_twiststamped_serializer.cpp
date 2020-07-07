/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_twiststamped_serializer.h"

int serialize_geometry_msgs_twiststamped(const geometry_msgs::TwistStamped &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_twist(send_data.twist, buffer + len);

  return len;
}

geometry_msgs::TwistStamped deserialize_geometry_msgs_twiststamped(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::TwistStamped msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.twist = deserialize_geometry_msgs_twist(buffer + length, len);
  length += len;

  return msg;
}

