/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _geometry_msgs_posestamped_serializer_h_
#define _geometry_msgs_posestamped_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/geometry_msgs/PoseStamped.h"

#include "cfs_serializer/std_msgs_header_serializer.h"
#include "cfs_serializer/geometry_msgs_pose_serializer.h"

int serialize_geometry_msgs_posestamped(const ::geometry_msgs::PoseStamped &send_data, unsigned char *buffer);
geometry_msgs::PoseStamped deserialize_geometry_msgs_posestamped(const unsigned char *buffer, unsigned int &length);

#endif // _geometry_msgs_posestamped_serializer_h_
