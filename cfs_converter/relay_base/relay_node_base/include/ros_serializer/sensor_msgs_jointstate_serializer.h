/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _sensor_msgs_jointstate_serializer_h_
#define _sensor_msgs_jointstate_serializer_h_

#include "sensor_msgs/JointState.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/std_msgs_header_serializer.h"

namespace ros_serializer {
namespace sensor_msgs {
class JointState {
public:
  int serialize(const ::sensor_msgs::JointState &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::std_msgs::Header header;
    len += header.serialize(send_data.header, buffer + len);

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

  ::sensor_msgs::JointState deserialize(const unsigned char *buffer,
                                        unsigned int &length) {
    ::sensor_msgs::JointState msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::std_msgs::Header header;
    msg.header = header.deserialize(buffer + length, len);
    length += len;

    std::vector<std::string> name =
        deserialize_array_string(buffer + length, len);
    for (size_t ii = 0; ii < name.size(); ii++) {
      msg.name.push_back(name[ii]);
    }
    length += len;

    std::vector<double> position =
        deserialize_array_double(buffer + length, len);
    for (size_t ii = 0; ii < position.size(); ii++) {
      msg.position.push_back(position[ii]);
    }
    length += len;

    std::vector<double> velocity =
        deserialize_array_double(buffer + length, len);
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
};
};
};

#endif // _sensor_msgs_jointstate_serializer_h_
