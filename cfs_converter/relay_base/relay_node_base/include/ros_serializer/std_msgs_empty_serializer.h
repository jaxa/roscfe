/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_empty_serializer_h_
#define _std_msgs_empty_serializer_h_

#include "std_msgs/Empty.h"

#include "relay_node/util.h"

namespace ros_serializer {
namespace std_msgs {
class Empty {
public:
  int serialize(const ::std_msgs::Empty &send_data, unsigned char *buffer) {
    unsigned int len = 0;

    return len;
  }

  ::std_msgs::Empty deserialize(const unsigned char *buffer,
                                unsigned int &length) {
    ::std_msgs::Empty msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    return msg;
  }
};
};
};

#endif // _std_msgs_empty_serializer_h_
