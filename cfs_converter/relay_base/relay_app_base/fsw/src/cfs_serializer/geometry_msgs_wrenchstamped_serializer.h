/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_wrenchstamped_serializer_h_
#define _geometry_msgs_wrenchstamped_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/WrenchStamped.h"

#include "cfs_serializer/std_msgs_header_serializer.h"
#include "cfs_serializer/geometry_msgs_wrench_serializer.h"

int serialize_geometry_msgs_wrenchstamped(const ::geometry_msgs::WrenchStamped &send_data, unsigned char *buffer);
geometry_msgs::WrenchStamped deserialize_geometry_msgs_wrenchstamped(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_wrenchstamped_serializer_h_
