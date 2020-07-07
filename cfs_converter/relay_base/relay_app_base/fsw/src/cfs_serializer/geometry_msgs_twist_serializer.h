/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_twist_serializer_h_
#define _geometry_msgs_twist_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Twist.h"

#include "cfs_serializer/geometry_msgs_vector3_serializer.h"

int serialize_geometry_msgs_twist(const ::geometry_msgs::Twist &send_data, unsigned char *buffer);
geometry_msgs::Twist deserialize_geometry_msgs_twist(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_twist_serializer_h_
