/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/sensor_msgs_image_serializer.h"

int serialize_sensor_msgs_image(const ::sensor_msgs::Image &send_data, unsigned char *buffer) {
  unsigned int ii;
  unsigned int len = 0;

  len += serialize_std_msgs_header(send_data.header, buffer + len);

  len += serialize_unsigned_int(send_data.height, buffer + len);

  len += serialize_unsigned_int(send_data.width, buffer + len);

  len += serialize_string(send_data.encoding, buffer + len);

  len += serialize_unsigned_char(send_data.is_bigendian, buffer + len);

  len += serialize_unsigned_int(send_data.step, buffer + len);

  std::vector<uint8> data;
  for (ii = 0; ii < send_data.data.size(); ii++) {
    data.push_back(send_data.data[ii]);
  }
  len += serialize_array_unsigned_char(data, buffer + len);

  return len;
}

sensor_msgs::Image deserialize_sensor_msgs_image(const unsigned char *buffer, unsigned int &length) {
  sensor_msgs::Image msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.header = deserialize_std_msgs_header(buffer + length, len);
  length += len;

  msg.height = deserialize_unsigned_int(buffer + length, len);
  length += len;

  msg.width = deserialize_unsigned_int(buffer + length, len);
  length += len;

  msg.encoding = deserialize_string(buffer + length, len);
  length += len;

  msg.is_bigendian = deserialize_unsigned_char(buffer + length, len);
  length += len;

  msg.step = deserialize_unsigned_int(buffer + length, len);
  length += len;

  std::vector<uint8> data = deserialize_array_unsigned_char(buffer + length, len);
  for (ii = 0; ii < data.size(); ii++) {
    msg.data.push_back(data[ii]);
  }
  length += len;

  return msg;
}

