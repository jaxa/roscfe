/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_multiarraylayout_serializer.h"

int serialize_std_msgs_multiarraylayout(const ::std_msgs::MultiArrayLayout &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_unsigned_int(send_data.dim.size(), buffer + len);
  for (size_t ii = 0; ii < send_data.dim.size(); ii++) {
    len += serialize_std_msgs_multiarraydimension(send_data.dim[ii], buffer + len);
  }

  len += serialize_unsigned_long(send_data.data_offset, buffer + len);

  return len;
}

std_msgs::MultiArrayLayout deserialize_std_msgs_multiarraylayout(const unsigned char *buffer, unsigned int &length) {
  std_msgs::MultiArrayLayout msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  size_t dim_max = deserialize_unsigned_int(buffer + length, len);
  length += len;
  for (size_t ii = 0; ii < dim_max; ii++) {
      msg.dim.push_back(deserialize_std_msgs_multiarraydimension(buffer + length, len));
      length += len;
  }

  msg.data_offset = deserialize_unsigned_long(buffer + length, len);
  length += len;

  return msg;
}
