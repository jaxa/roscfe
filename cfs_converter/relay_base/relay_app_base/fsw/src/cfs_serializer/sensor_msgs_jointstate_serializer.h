/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _sensor_msgs_jointstate_serializer_h_
#define _sensor_msgs_jointstate_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/sensor_msgs/JointState.h"

#include "cfs_serializer/std_msgs_header_serializer.h"

int serialize_sensor_msgs_jointstate(const ::sensor_msgs::JointState &send_data, unsigned char *buffer);
sensor_msgs::JointState deserialize_sensor_msgs_jointstate(const unsigned char *buffer, unsigned int &length);

#endif // _sensor_msgs_jointstate_serializer_h_
