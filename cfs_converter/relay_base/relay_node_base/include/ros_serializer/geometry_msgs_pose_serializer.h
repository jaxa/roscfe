/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_pose_serializer_h_
#define _geometry_msgs_pose_serializer_h_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_point_serializer.h"
#include "ros_serializer/geometry_msgs_quaternion_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class Pose {
public:
  int serialize(const ::geometry_msgs::Pose &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::geometry_msgs::Point position;
    len += position.serialize(send_data.position, buffer + len);

    ros_serializer::geometry_msgs::Quaternion orientation;
    len += orientation.serialize(send_data.orientation, buffer + len);

    return len;
  }

  ::geometry_msgs::Pose deserialize(const unsigned char *buffer,
                                    unsigned int &length) {
    ::geometry_msgs::Pose msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::geometry_msgs::Point position;
    msg.position = position.deserialize(buffer + length, len);
    length += len;

    ros_serializer::geometry_msgs::Quaternion orientation;
    msg.orientation = orientation.deserialize(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_pose_serializer_h_
