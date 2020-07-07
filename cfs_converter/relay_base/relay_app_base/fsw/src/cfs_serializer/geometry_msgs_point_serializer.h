/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_point_serializer_h_
#define _geometry_msgs_point_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Point.h"

int serialize_geometry_msgs_point(const geometry_msgs::Point &send_data, unsigned char *buffer);
geometry_msgs::Point deserialize_geometry_msgs_point(const unsigned char *buffer, unsigned int &length);


#endif // _geometry_msgs_point_serializer_h_
