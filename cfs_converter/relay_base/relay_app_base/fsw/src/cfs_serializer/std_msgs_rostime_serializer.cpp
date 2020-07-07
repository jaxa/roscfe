/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_rostime_serializer_h_
#define _std_msgs_rostime_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/std_msgs/RosTime.h"

int serialize_std_msgs_rostime(const RosTime &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_unsigned_long(send_data.sec, buffer + len);
  len += serialize_unsigned_long(send_data.nsec, buffer + len);

  return len;
}

RosTime deserialize_std_msgs_rostime(const unsigned char *buffer, unsigned int &length) {
  RosTime msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.sec = deserialize_unsigned_long(buffer + length, len);
  length += len;

  msg.nsec = deserialize_unsigned_long(buffer + length, len);
  length += len;

  return msg;
}

#endif // _std_msgs_rostime_serializer_h_
