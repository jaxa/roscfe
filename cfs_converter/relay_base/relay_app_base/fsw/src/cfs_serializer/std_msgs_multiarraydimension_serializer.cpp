/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_multiarraydimension_serializer.h"

int serialize_std_msgs_multiarraydimension(const ::std_msgs::MultiArrayDimension &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_string(send_data.label, buffer + len);
  len += serialize_unsigned_long(send_data.size, buffer + len);
  len += serialize_unsigned_long(send_data.stride, buffer + len);

  return len;
}

std_msgs::MultiArrayDimension deserialize_std_msgs_multiarraydimension(const unsigned char *buffer, unsigned int &length) {
  std_msgs::MultiArrayDimension msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.label = deserialize_string(buffer + length, len);
  length += len;

  msg.size = deserialize_unsigned_long(buffer + length, len);
  length += len;

  msg.stride = deserialize_unsigned_long(buffer + length, len);
  length += len;

  return msg;
}
