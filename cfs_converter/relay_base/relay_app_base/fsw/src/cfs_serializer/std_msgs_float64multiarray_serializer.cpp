/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_float64multiarray_serializer.h"

#include "cfe.h"
#include "util.h"

#include "../convert/std_msgs/Float64MultiArray.h"

int serialize_std_msgs_float64multiarray(const ::std_msgs::Float64MultiArray &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_std_msgs_multiarraylayout(send_data.layout, buffer + len);
  
  std::vector<float> data;
  for (size_t ii = 0; ii < send_data.data.size(); ii++) {
    data.push_back(send_data.data[ii]);
  }
  len += serialize_array_float(data, buffer + len);

  return len;
}
std_msgs::Float64MultiArray deserialize_std_msgs_float64multiarray(const unsigned char *buffer, unsigned int &length) {
  std_msgs::Float64MultiArray msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.layout = deserialize_std_msgs_multiarraylayout(buffer + length, len);
  length += len;

  std::vector<float> data = deserialize_array_float(buffer + length, len);
  for (size_t ii = 0; ii < data.size(); ii++) {
    msg.data.push_back(data[ii]);
  }
  length += len;

  return msg;
}

