/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_float64multiarray_serializer_h_
#define _std_msgs_float64multiarray_serializer_h_

#include "std_msgs/Float64MultiArray.h"

#include "relay_node/util.h"
#include <vector>

#include "ros_serializer/std_msgs_multiarraylayout_serializer.h"

namespace ros_serializer {
namespace std_msgs {
class Float64MultiArray {
public:
  int serialize(const ::std_msgs::Float64MultiArray &send_data,
                unsigned char *buffer) {
    unsigned int len = 0;

    ros_serializer::std_msgs::MultiArrayLayout layout;
    len += layout.serialize(send_data.layout, buffer + len);

    std::vector<float> data;
    for (size_t ii = 0; ii < send_data.data.size(); ii++) {
      data.push_back(send_data.data[ii]);
    }
    len += serialize_array_float(data, buffer + len);

    return len;
  }

  ::std_msgs::Float64MultiArray deserialize(const unsigned char *buffer,
                                            unsigned int &length) {
    ::std_msgs::Float64MultiArray msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::std_msgs::MultiArrayLayout layout;
    msg.layout = layout.deserialize(buffer + length, len);
    length += len;

    std::vector<float> data = deserialize_array_float(buffer + length, len);
    for (size_t ii = 0; ii < data.size(); ii++) {
      msg.data.push_back(data[ii]);
    }
    length += len;

    return msg;
  }
};
};
};

#endif // _std_msgs_float64multiarray_serializer_h_
