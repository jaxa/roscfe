/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _nav_msgs_odometry_serializer_h_
#define _nav_msgs_odometry_serializer_h_

#include "nav_msgs/Odometry.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_posewithcovariance_serializer.h"
#include "ros_serializer/geometry_msgs_twistwithcovariance_serializer.h"
#include "ros_serializer/std_msgs_header_serializer.h"

namespace ros_serializer {
namespace nav_msgs {
class Odometry {
public:
  int serialize(const ::nav_msgs::Odometry &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::std_msgs::Header header;
    len += header.serialize(send_data.header, buffer + len);

    ros_serializer::geometry_msgs::PoseWithCovariance pose;
    len += pose.serialize(send_data.pose, buffer + len);

    ros_serializer::geometry_msgs::TwistWithCovariance twist;
    len += twist.serialize(send_data.twist, buffer + len);

    return len;
  }

  ::nav_msgs::Odometry deserialize(const unsigned char *buffer,
                                   unsigned int &length) {
    ::nav_msgs::Odometry msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::std_msgs::Header header;
    msg.header = header.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::PoseWithCovariance pose;
    msg.pose = pose.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::TwistWithCovariance twist;
    msg.twist = twist.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _nav_msgs_odometry_serializer_h_