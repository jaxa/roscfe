/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_posewithcovariance_serializer_h_
#define _geometry_msgs_posewithcovariance_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/PoseWithCovariance.h"

#include "cfs_serializer/geometry_msgs_pose_serializer.h"

int serialize_geometry_msgs_posewithcovariance(const geometry_msgs::PoseWithCovariance &send_data, unsigned char *buffer);
geometry_msgs::PoseWithCovariance deserialize_geometry_msgs_posewithcovariance(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_posewithcovariance_serializer_h_
