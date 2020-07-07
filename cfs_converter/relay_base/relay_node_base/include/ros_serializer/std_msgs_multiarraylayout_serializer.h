/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_multiarraylayout_serializer_h_
#define _std_msgs_multiarraylayout_serializer_h_

#include "std_msgs/MultiArrayLayout.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/std_msgs_multiarraydimension_serializer.h"

namespace ros_serializer {
namespace std_msgs {
class MultiArrayLayout {
public:
  int serialize(const ::std_msgs::MultiArrayLayout &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    len += serialize_unsigned_int(send_data.dim.size(), buffer + len);
    ros_serializer::std_msgs::MultiArrayDimension dim;
    for (size_t ii = 0; ii < send_data.dim.size(); ii++) {
      len += dim.serialize(send_data.dim[ii], buffer + len);
    }

    len += serialize_unsigned_long(send_data.data_offset, buffer + len);

    return len;
  }

  ::std_msgs::MultiArrayLayout deserialize(const unsigned char *buffer,
                                           unsigned int &length) {
    ::std_msgs::MultiArrayLayout msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    size_t dim_max = deserialize_unsigned_int(buffer + length, len);
    length += len;
    ros_serializer::std_msgs::MultiArrayDimension dim;
    for (size_t ii = 0; ii < dim_max; ii++) {
      msg.dim.push_back(dim.deserialize(buffer + length, len));
      length += len;
    }

    msg.data_offset = deserialize_unsigned_long(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _std_msgs_multiarraylayout_serializer_h_
