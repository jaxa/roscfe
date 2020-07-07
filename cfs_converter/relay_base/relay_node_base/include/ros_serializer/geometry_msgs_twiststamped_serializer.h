/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_twiststamped_serializer_h_
#define _geometry_msgs_twiststamped_serializer_h_

#include "geometry_msgs/TwistStamped.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_twist_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class TwistStamped {
public:
  int serialize(const ::geometry_msgs::TwistStamped &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::geometry_msgs::Twist twist;
    len += twist.serialize(send_data.twist, buffer + len);

    return len;
  }

  ::geometry_msgs::TwistStamped deserialize(const unsigned char *buffer,
                                            unsigned int &length) {
    ::geometry_msgs::TwistStamped msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::geometry_msgs::Twist twist;
    msg.twist = twist.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_twiststamped_serializer_h_
