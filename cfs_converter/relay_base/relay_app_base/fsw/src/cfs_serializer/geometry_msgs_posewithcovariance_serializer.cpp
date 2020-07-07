/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_posewithcovariance_serializer.h"

int serialize_geometry_msgs_posewithcovariance(const geometry_msgs::PoseWithCovariance &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_pose(send_data.pose, buffer + len);
  
  std::vector<float> covariance;
  for (size_t ii = 0; ii < 36; ii++) {
    covariance.push_back(send_data.covariance[ii]);
  }
  len += serialize_array_float(covariance, buffer + len);

  return len;
}

geometry_msgs::PoseWithCovariance deserialize_geometry_msgs_posewithcovariance(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::PoseWithCovariance msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.pose = deserialize_geometry_msgs_pose(buffer + length, len);
  length += len;

  std::vector<float> covariance = deserialize_array_float(buffer + length, len);
  for (size_t ii = 0; ii < covariance.size(); ii++) {
    msg.covariance[ii] = covariance[ii];
  }
  length += len;

  return msg;
}

