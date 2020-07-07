/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_pose_serializer.h"

int serialize_geometry_msgs_pose(const geometry_msgs::Pose &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_point(send_data.position, buffer + len);

  len += serialize_geometry_msgs_quaternion(send_data.orientation, buffer + len);

  return len;
}

geometry_msgs::Pose deserialize_geometry_msgs_pose(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::Pose msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.position = deserialize_geometry_msgs_point(buffer + length, len);
  length += len;

  msg.orientation = deserialize_geometry_msgs_quaternion(buffer + length, len);
  length += len;

  return msg;
}
