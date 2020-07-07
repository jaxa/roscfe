/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_string_serializer_h_
#define _std_msgs_string_serializer_h_

#include "std_msgs/String.h"

#include "relay_node/util.h"
#include <vector>

namespace ros_serializer {
namespace std_msgs {
class String {
public:
  int serialize(const ::std_msgs::String &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    len += serialize_string(send_data.data, buffer + len);

    return len;
  }

  ::std_msgs::String deserialize(const unsigned char *buffer,
                                 unsigned int &length) {
    ::std_msgs::String msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    msg.data = deserialize_string(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _std_msgs_string_serializer_h_
