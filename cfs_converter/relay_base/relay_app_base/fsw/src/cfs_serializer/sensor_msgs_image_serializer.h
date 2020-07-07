/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _sensor_msgs_image_serializer_h_
#define _sensor_msgs_image_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/sensor_msgs/Image.h"

#include "cfs_serializer/std_msgs_header_serializer.h"

int serialize_sensor_msgs_image(const ::sensor_msgs::Image &send_data, unsigned char *buffer);
sensor_msgs::Image deserialize_sensor_msgs_image(const unsigned char *buffer, unsigned int &length);

#endif // _sensor_msgs_image_serializer_h_
