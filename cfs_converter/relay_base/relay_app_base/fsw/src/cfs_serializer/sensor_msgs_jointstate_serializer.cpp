/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/sensor_msgs_jointstate_serializer.h"

int serialize_sensor_msgs_jointstate(const ::sensor_msgs::JointState &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  len += serialize_std_msgs_header(send_data.header, buffer + len);

  std::vector<std::string> name;
  for (size_t ii = 0; ii < send_data.name.size(); ii++) {
    name.push_back(send_data.name[ii]);
  }
  len += serialize_array_string(name, buffer + len);

  std::vector<double> position;
  for (size_t ii = 0; ii < send_data.position.size(); ii++) {
    position.push_back(send_data.position[ii]);
  }
  len += serialize_array_double(position, buffer + len);

  std::vector<double> velocity;
  for (size_t ii = 0; ii < send_data.velocity.size(); ii++) {
    velocity.push_back(send_data.velocity[ii]);
  }
  len += serialize_array_double(velocity, buffer + len);

  std::vector<double> effort;
  for (size_t ii = 0; ii < send_data.effort.size(); ii++) {
    effort.push_back(send_data.effort[ii]);
  }
  len += serialize_array_double(effort, buffer + len);

  return len;
}

sensor_msgs::JointState deserialize_sensor_msgs_jointstate(const unsigned char *buffer, unsigned int &length) {
  sensor_msgs::JointState msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  msg.header = deserialize_std_msgs_header(buffer + length, len);
  length += len;

  std::vector<std::string> name = deserialize_array_string(buffer + length, len);
  for (size_t ii = 0; ii < name.size(); ii++) {
    msg.name.push_back(name[ii]);
  }
  length += len;

  std::vector<double> position = deserialize_array_double(buffer + length, len);
  for (size_t ii = 0; ii < position.size(); ii++) {
    msg.position.push_back(position[ii]);
  }
  length += len;

  std::vector<double> velocity = deserialize_array_double(buffer + length, len);
  for (size_t ii = 0; ii < velocity.size(); ii++) {
    msg.velocity.push_back(velocity[ii]);
  }
  length += len;

  std::vector<double> effort = deserialize_array_double(buffer + length, len);
  for (size_t ii = 0; ii < effort.size(); ii++) {
    msg.effort.push_back(effort[ii]);
  }
  length += len;

  return msg;
}

