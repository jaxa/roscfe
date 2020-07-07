/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_wrench_serializer_h_
#define _geometry_msgs_wrench_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Wrench.h"

#include "cfs_serializer/geometry_msgs_vector3_serializer.h"

int serialize_geometry_msgs_wrench(const ::geometry_msgs::Wrench &send_data, unsigned char *buffer);
geometry_msgs::Wrench deserialize_geometry_msgs_wrench(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_wrench_serializer_h_
