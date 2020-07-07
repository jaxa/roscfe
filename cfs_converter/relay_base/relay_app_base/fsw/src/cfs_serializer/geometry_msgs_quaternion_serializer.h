/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_quaternion_serializer_h_
#define _geometry_msgs_quaternion_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Quaternion.h"

int serialize_geometry_msgs_quaternion(const ::geometry_msgs::Quaternion &send_data, unsigned char *buffer);
geometry_msgs::Quaternion deserialize_geometry_msgs_quaternion(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_quaternion_serializer_h_