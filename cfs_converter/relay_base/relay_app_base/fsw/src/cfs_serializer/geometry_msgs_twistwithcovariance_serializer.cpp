/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/geometry_msgs_twistwithcovariance_serializer.h"

int serialize_geometry_msgs_twistwithcovariance(const ::geometry_msgs::TwistWithCovariance &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_geometry_msgs_twist(send_data.twist, buffer + len);
  
  std::vector<float> covariance;
  for (size_t ii = 0; ii < 36; ii++) {
    covariance.push_back(send_data.covariance[ii]);
  }
  len += serialize_array_float(covariance, buffer + len);

  return len;
}
geometry_msgs::TwistWithCovariance deserialize_geometry_msgs_twistwithcovariance(const unsigned char *buffer, unsigned int &length) {
  geometry_msgs::TwistWithCovariance msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.twist = deserialize_geometry_msgs_twist(buffer + length, len);
  length += len;

  std::vector<float> covariance = deserialize_array_float(buffer + length, len);
  for (size_t ii = 0; ii < covariance.size(); ii++) {
    msg.covariance[ii] = covariance[ii];
  }
  length += len;

  return msg;
}

