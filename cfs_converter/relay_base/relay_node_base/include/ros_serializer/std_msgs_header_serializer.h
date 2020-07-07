/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_header_serializer_h_
#define _std_msgs_header_serializer_h_

#include "std_msgs/Header.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/std_msgs_rostime_serializer.h"

namespace ros_serializer {
namespace std_msgs {
class Header {
public:
  int serialize(const ::std_msgs::Header &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    len += serialize_unsigned_long(send_data.seq, buffer + len);

    ros_serializer::std_msgs::RosTime stamp;
    len += stamp.serialize(send_data.stamp, buffer + len);

    len += serialize_string(send_data.frame_id, buffer + len);

    return len;
  }

  ::std_msgs::Header deserialize(const unsigned char *buffer,
                                 unsigned int &length) {
    ::std_msgs::Header msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    msg.seq = deserialize_unsigned_long(buffer + length, len);
    length += len;

    ros_serializer::std_msgs::RosTime stamp;
    msg.stamp = stamp.deserialize(buffer + length, len);
    length += len;

    msg.frame_id = deserialize_string(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _std_msgs_header_serializer_h_
