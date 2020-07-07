/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_vector3_serializer_h_
#define _geometry_msgs_vector3_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Vector3.h"

int serialize_geometry_msgs_vector3(const ::geometry_msgs::Vector3 &send_data, unsigned char *buffer);
geometry_msgs::Vector3 deserialize_geometry_msgs_vector3(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_vector3_serializer_h_
