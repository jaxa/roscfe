/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_posewithcovariance_serializer_h_
#define _geometry_msgs_posewithcovariance_serializer_h_

#include "geometry_msgs/PoseWithCovariance.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/geometry_msgs_pose_serializer.h"

namespace ros_serializer {
namespace geometry_msgs {
class PoseWithCovariance {
public:
  int serialize(const ::geometry_msgs::PoseWithCovariance &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::geometry_msgs::Pose pose;
    len += pose.serialize(send_data.pose, buffer + len);

    std::vector<float> covariance;
    for (size_t ii = 0; ii < send_data.covariance.size(); ii++) {
      covariance.push_back(send_data.covariance[ii]);
    }
    len += serialize_array_float(covariance, buffer + len);

    return len;
  }

  ::geometry_msgs::PoseWithCovariance deserialize(const unsigned char *buffer,
                                                  unsigned int &length) {
    ::geometry_msgs::PoseWithCovariance msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::geometry_msgs::Pose pose;
    msg.pose = pose.deserialize(buffer + length, len);
    length += len;

    std::vector<float> covariance =
        deserialize_array_float(buffer + length, len);
    for (size_t ii = 0; ii < covariance.size(); ii++) {
      msg.covariance[ii] = covariance[ii];
    }
    length += len;

    return msg;
  }
};
};
};

#endif // _geometry_msgs_posewithcovariance_serializer_h_
