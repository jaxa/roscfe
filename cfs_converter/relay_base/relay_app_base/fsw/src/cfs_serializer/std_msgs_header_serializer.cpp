/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_header_serializer.h"

int serialize_std_msgs_header(const ::std_msgs::Header &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_unsigned_long(send_data.seq, buffer + len);

  len += serialize_std_msgs_rostime(send_data.stamp, buffer + len);
  
  len += serialize_string(send_data.frame_id, buffer + len);

  return len;
}

std_msgs::Header deserialize_std_msgs_header(const unsigned char *buffer, unsigned int &length) {
  std_msgs::Header msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.seq = deserialize_unsigned_long(buffer + length, len);
  length += len;

  msg.stamp = deserialize_std_msgs_rostime(buffer + length, len);
  length += len;

  msg.frame_id = deserialize_string(buffer + length, len);
  length += len;

  return msg;
}
