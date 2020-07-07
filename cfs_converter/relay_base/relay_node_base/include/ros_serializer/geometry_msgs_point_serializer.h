/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_point_serializer_h_
#define _geometry_msgs_point_serializer_h_

#include "geometry_msgs/Point.h"

#include "relay_node/util.h"
#include <vector>

namespace ros_serializer {
namespace geometry_msgs {
class Point {
public:
  int serialize(const ::geometry_msgs::Point &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    len += serialize_float(send_data.x, buffer + len);
    len += serialize_float(send_data.y, buffer + len);
    len += serialize_float(send_data.z, buffer + len);

    return len;
  }

  ::geometry_msgs::Point deserialize(const unsigned char *buffer,
                                     unsigned int &length) {
    ::geometry_msgs::Point msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    msg.x = deserialize_float(buffer + length, len);
    length += len;

    msg.y = deserialize_float(buffer + length, len);
    length += len;

    msg.z = deserialize_float(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_point_serializer_h_
