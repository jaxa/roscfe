/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _ardrone_autonomy_navdata_serializer_h_
#define _ardrone_autonomy_navdata_serializer_h_

#include "ardrone_autonomy/Navdata.h"
#include "relay_node/util.h"

#include "ros_serializer/std_msgs_header_serializer.h"

namespace ros_serializer {
namespace ardrone_autonomy {
class Navdata {
public:
  int serialize(const ::ardrone_autonomy::Navdata &send_data,
                unsigned char *buffer) {
    unsigned int ii;
    unsigned int len = 0;

    ros_serializer::std_msgs::Header header;
    len += header.serialize(send_data.header, buffer + len);

    len += serialize_float(send_data.batteryPercent, buffer + len);

    len += serialize_unsigned_int(send_data.state, buffer + len);

    len += serialize_int(send_data.magX, buffer + len);
    len += serialize_int(send_data.magY, buffer + len);
    len += serialize_int(send_data.magZ, buffer + len);

    len += serialize_int(send_data.pressure, buffer + len);

    len += serialize_int(send_data.temp, buffer + len);

    len += serialize_float(send_data.wind_speed, buffer + len);
    len += serialize_float(send_data.wind_angle, buffer + len);
    len += serialize_float(send_data.wind_comp_angle, buffer + len);

    len += serialize_float(send_data.rotX, buffer + len);
    len += serialize_float(send_data.rotY, buffer + len);
    len += serialize_float(send_data.rotZ, buffer + len);

    len += serialize_int(send_data.altd, buffer + len);

    len += serialize_float(send_data.vx, buffer + len);
    len += serialize_float(send_data.vy, buffer + len);
    len += serialize_float(send_data.vz, buffer + len);

    len += serialize_float(send_data.ax, buffer + len);
    len += serialize_float(send_data.ay, buffer + len);
    len += serialize_float(send_data.az, buffer + len);

    len += serialize_unsigned_char(send_data.motor1, buffer + len);
    len += serialize_unsigned_char(send_data.motor2, buffer + len);
    len += serialize_unsigned_char(send_data.motor3, buffer + len);
    len += serialize_unsigned_char(send_data.motor4, buffer + len);

    len += serialize_unsigned_int(send_data.tags_count, buffer + len);

    std::vector<uint32_t> tags_type;
    for (ii = 0; ii < send_data.tags_type.size(); ii++) {
      tags_type.push_back(send_data.tags_type[ii]);
    }
    len += serialize_array_unsigned_int(tags_type, buffer + len);

    std::vector<uint32_t> tags_xc;
    for (ii = 0; ii < send_data.tags_xc.size(); ii++) {
      tags_xc.push_back(send_data.tags_xc[ii]);
    }
    len += serialize_array_unsigned_int(tags_xc, buffer + len);

    std::vector<uint32_t> tags_yc;
    for (ii = 0; ii < send_data.tags_yc.size(); ii++) {
      tags_yc.push_back(send_data.tags_yc[ii]);
    }
    len += serialize_array_unsigned_int(tags_yc, buffer + len);

    std::vector<uint32_t> tags_width;
    for (ii = 0; ii < send_data.tags_width.size(); ii++) {
      tags_width.push_back(send_data.tags_width[ii]);
    }
    len += serialize_array_unsigned_int(tags_width, buffer + len);

    std::vector<uint32_t> tags_height;
    for (ii = 0; ii < send_data.tags_height.size(); ii++) {
      tags_height.push_back(send_data.tags_height[ii]);
    }
    len += serialize_array_unsigned_int(tags_height, buffer + len);

    std::vector<float> tags_orientation;
    for (ii = 0; ii < send_data.tags_orientation.size(); ii++) {
      tags_orientation.push_back(send_data.tags_orientation[ii]);
    }
    len += serialize_array_float(tags_orientation, buffer + len);

    std::vector<float> tags_distance;
    for (ii = 0; ii < send_data.tags_distance.size(); ii++) {
      tags_distance.push_back(send_data.tags_distance[ii]);
    }
    len += serialize_array_float(tags_distance, buffer + len);

    len += serialize_float(send_data.tm, buffer + len);

    return len;
  }

  ::ardrone_autonomy::Navdata deserialize(const unsigned char *buffer,
                                          unsigned int &length) {
    ::ardrone_autonomy::Navdata msg;
    unsigned int ii;
    unsigned int len;
    length = 0;

    ros_serializer::std_msgs::Header header;
    msg.header = header.deserialize(buffer + length, len);
    length += len;

    msg.batteryPercent = deserialize_float(buffer + length, len);
    length += len;

    msg.state = deserialize_unsigned_int(buffer + length, len);
    length += len;

    msg.magX = deserialize_int(buffer + length, len);
    length += len;
    msg.magY = deserialize_int(buffer + length, len);
    length += len;
    msg.magZ = deserialize_int(buffer + length, len);
    length += len;

    msg.pressure = deserialize_int(buffer + length, len);
    length += len;

    msg.temp = deserialize_int(buffer + length, len);
    length += len;

    msg.wind_speed = deserialize_float(buffer + length, len);
    length += len;
    msg.wind_angle = deserialize_float(buffer + length, len);
    length += len;
    msg.wind_comp_angle = deserialize_float(buffer + length, len);
    length += len;

    msg.rotX = deserialize_float(buffer + length, len);
    length += len;
    msg.rotY = deserialize_float(buffer + length, len);
    length += len;
    msg.rotZ = deserialize_float(buffer + length, len);
    length += len;

    msg.altd = deserialize_int(buffer + length, len);
    length += len;

    msg.vx = deserialize_float(buffer + length, len);
    length += len;
    msg.vy = deserialize_float(buffer + length, len);
    length += len;
    msg.vz = deserialize_float(buffer + length, len);
    length += len;

    msg.ax = deserialize_float(buffer + length, len);
    length += len;
    msg.ay = deserialize_float(buffer + length, len);
    length += len;
    msg.az = deserialize_float(buffer + length, len);
    length += len;

    msg.motor1 = deserialize_unsigned_char(buffer + length, len);
    length += len;
    msg.motor2 = deserialize_unsigned_char(buffer + length, len);
    length += len;
    msg.motor3 = deserialize_unsigned_char(buffer + length, len);
    length += len;
    msg.motor4 = deserialize_unsigned_char(buffer + length, len);
    length += len;

    msg.tags_count = deserialize_unsigned_int(buffer + length, len);
    length += len;

    std::vector<uint32_t> tags_type =
        deserialize_array_unsigned_int(buffer + length, len);
    for (ii = 0; ii < tags_type.size(); ii++) {
      msg.tags_type.push_back(tags_type[ii]);
    }
    length += len;

    std::vector<uint32_t> tags_xc =
        deserialize_array_unsigned_int(buffer + length, len);
    for (ii = 0; ii < tags_xc.size(); ii++) {
      msg.tags_xc.push_back(tags_xc[ii]);
    }
    length += len;

    std::vector<uint32_t> tags_yc =
        deserialize_array_unsigned_int(buffer + length, len);
    for (ii = 0; ii < tags_yc.size(); ii++) {
      msg.tags_yc.push_back(tags_yc[ii]);
    }
    length += len;

    std::vector<uint32_t> tags_width =
        deserialize_array_unsigned_int(buffer + length, len);
    for (ii = 0; ii < tags_width.size(); ii++) {
      msg.tags_width.push_back(tags_width[ii]);
    }
    length += len;

    std::vector<uint32_t> tags_height =
        deserialize_array_unsigned_int(buffer + length, len);
    for (ii = 0; ii < tags_height.size(); ii++) {
      msg.tags_height.push_back(tags_height[ii]);
    }
    length += len;

    std::vector<float> tags_orientation =
        deserialize_array_float(buffer + length, len);
    for (ii = 0; ii < tags_orientation.size(); ii++) {
      msg.tags_orientation.push_back(tags_orientation[ii]);
    }
    length += len;

    std::vector<float> tags_distance =
        deserialize_array_float(buffer + length, len);
    for (ii = 0; ii < tags_distance.size(); ii++) {
      msg.tags_distance.push_back(tags_distance[ii]);
    }
    length += len;

    msg.tm = deserialize_float(buffer + length, len);
    length += len;

    return msg;
  }
};
};
};

#endif // _ardrone_autonomy_navdata_serializer_h_
