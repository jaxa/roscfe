/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_wrenchstamped_serializer_h_
#define _geometry_msgs_wrenchstamped_serializer_h_

#include "geometry_msgs/WrenchStamped.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_wrench_serializer.h"
#include "ros_serializer/std_msgs_header_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class WrenchStamped {
public:
  int serialize(const ::geometry_msgs::WrenchStamped &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::std_msgs::Header header;
    len += header.serialize(send_data.header, buffer + len);

    ros_serializer::geometry_msgs::Wrench wrench;
    len += wrench.serialize(send_data.wrench, buffer + len);

    return len;
  }

  ::geometry_msgs::WrenchStamped deserialize(const unsigned char *buffer,
                                             unsigned int &length) {
    ::geometry_msgs::WrenchStamped msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::std_msgs::Header header;
    msg.header = header.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::Wrench wrench;
    msg.wrench = wrench.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_wrenchstamped_serializer_h_
