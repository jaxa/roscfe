/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_string_serializer.h"

int serialize_std_msgs_string(const ::std_msgs::String &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_string(send_data.data, buffer + len);

  return len;
}

std_msgs::String deserialize_std_msgs_string(const unsigned char *buffer, unsigned int &length) {
  std_msgs::String msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.data = deserialize_string(buffer + length, len);
  length += len;

  return msg;
}
