/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "define.h"
#include "util.h"

/**
 * Deserialize communication header
 * @param buffer Data to deserialize
 * @return Deserialize result
 */
RosCfeNwBridgeHeader deserialize_ros_cfe_header(unsigned char *buffer) {
  RosCfeNwBridgeHeader header;
  unsigned int len;
  unsigned int read_len = 0;

  header.body_len = deserialize_unsigned_int(buffer + read_len, len);
  read_len += len;
  header.unix_time = deserialize_double(buffer + read_len, len);
  read_len += len;
  header.sender = deserialize_unsigned_char(buffer + read_len, len);
  read_len += len;
  header.send_no = deserialize_unsigned_char(buffer + read_len, len);
  read_len += len;
  header.msg_id = deserialize_unsigned_short(buffer + read_len, len);
  read_len += len;

  return header;
};

/**
 * Serialize communication header
 * @param header Data to serialize
 * @param buffer Serialize result
 * @return Data size
 */
int serialize_ros_cfe_header(const RosCfeNwBridgeHeader &header, unsigned char *buffer) {
  unsigned int len;
  unsigned int write_len = 0;

  len = serialize_unsigned_int(header.body_len, buffer + write_len);
  write_len += len;
  len = serialize_double(header.unix_time, buffer + write_len);
  write_len += len;
  len = serialize_unsigned_char(header.sender, buffer + write_len);
  write_len += len;
  len = serialize_unsigned_char(header.send_no, buffer + write_len);
  write_len += len;
  len = serialize_unsigned_short(header.msg_id, buffer + write_len);
  write_len += len;

  return write_len;
};
