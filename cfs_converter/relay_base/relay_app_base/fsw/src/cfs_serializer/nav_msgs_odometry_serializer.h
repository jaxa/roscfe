/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _nav_msgs_odometry_serializer_h_
#define _nav_msgs_odometry_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/nav_msgs/Odometry.h"

#include "cfs_serializer/std_msgs_header_serializer.h"
#include "cfs_serializer/geometry_msgs_posewithcovariance_serializer.h"
#include "cfs_serializer/geometry_msgs_twistwithcovariance_serializer.h"

int serialize_nav_msgs_odometry(const ::nav_msgs::Odometry &send_data, unsigned char *buffer);
nav_msgs::Odometry deserialize_nav_msgs_odometry(const unsigned char *buffer, unsigned int &length);

#endif // _nav_msgs_odometry_serializer_h_