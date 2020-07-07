/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_twist_serializer_h_
#define _geometry_msgs_twist_serializer_h_

#include "geometry_msgs/Twist.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_vector3_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class Twist {
public:
  int serialize(const ::geometry_msgs::Twist &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::geometry_msgs::Vector3 linear;
    len += linear.serialize(send_data.linear, buffer + len);

    ros_serializer::geometry_msgs::Vector3 angular;
    len += angular.serialize(send_data.angular, buffer + len);

    return len;
  }

  ::geometry_msgs::Twist deserialize(const unsigned char *buffer,
                                     unsigned int &length) {
    ::geometry_msgs::Twist msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::geometry_msgs::Vector3 linear;
    msg.linear = linear.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::Vector3 angular;
    msg.angular = angular.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_twist_serializer_h_
