/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_twistwithcovariance_serializer_h_
#define _geometry_msgs_twistwithcovariance_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/TwistWithCovariance.h"

#include "cfs_serializer/geometry_msgs_twist_serializer.h"

int serialize_geometry_msgs_twistwithcovariance(const ::geometry_msgs::TwistWithCovariance &send_data, unsigned char *buffer);
geometry_msgs::TwistWithCovariance deserialize_geometry_msgs_twistwithcovariance(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_twistwithcovariance_serializer_h_
