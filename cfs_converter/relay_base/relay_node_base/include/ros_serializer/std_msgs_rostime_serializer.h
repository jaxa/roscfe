/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_rostime_serializer_h_
#define _std_msgs_rostime_serializer_h_

#include "std_msgs/Time.h"

#include "relay_node/util.h"
#include <vector>

namespace ros_serializer {
namespace std_msgs {
class RosTime {
public:
  int serialize(const ros::Time &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    len += serialize_unsigned_long(send_data.sec, buffer + len);
    len += serialize_unsigned_long(send_data.nsec, buffer + len);

    return len;
  }

  ros::Time deserialize(const unsigned char *buffer, unsigned int &length) {
    ros::Time msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    msg.sec = deserialize_unsigned_long(buffer + length, len);
    length += len;

    msg.nsec = deserialize_unsigned_long(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _std_msgs_rostime_serializer_h_
