/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_pose_serializer_h_
#define _geometry_msgs_pose_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/Pose.h"

#include "cfs_serializer/geometry_msgs_point_serializer.h"
#include "cfs_serializer/geometry_msgs_quaternion_serializer.h"

int serialize_geometry_msgs_pose(const geometry_msgs::Pose &send_data, unsigned char *buffer);
geometry_msgs::Pose deserialize_geometry_msgs_pose(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_pose_serializer_h_
