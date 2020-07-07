/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_posestamped_serializer.h"

int serialize_geometry_msgs_posestamped(const ::geometry_msgs::PoseStamped &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_std_msgs_header(send_data.header, buffer + len);

  len += serialize_geometry_msgs_pose(send_data.pose, buffer + len);

  return len;
}

geometry_msgs::PoseStamped deserialize_geometry_msgs_posestamped(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::PoseStamped msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.header = deserialize_std_msgs_header(buffer + length, len);
  length += len;

  msg.pose = deserialize_geometry_msgs_pose(buffer + length, len);
  length += len;

  return msg;
}
