/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_wrench_serializer_h_
#define _geometry_msgs_wrench_serializer_h_

#include "geometry_msgs/Wrench.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_vector3_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class Wrench {
public:
  int serialize(const ::geometry_msgs::Wrench &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::geometry_msgs::Vector3 force;
    len += force.serialize(send_data.force, buffer + len);

    ros_serializer::geometry_msgs::Vector3 torque;
    len += torque.serialize(send_data.torque, buffer + len);

    return len;
  }

  ::geometry_msgs::Wrench deserialize(const unsigned char *buffer,
                                      unsigned int &length) {
    ::geometry_msgs::Wrench msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::geometry_msgs::Vector3 force;
    msg.force = force.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::Vector3 torque;
    msg.torque = torque.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_wrench_serializer_h_
