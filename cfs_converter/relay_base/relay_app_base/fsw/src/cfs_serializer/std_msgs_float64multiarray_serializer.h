/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_float64multiarray_serializer_h_
#define _std_msgs_float64multiarray_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/std_msgs/Float64MultiArray.h"

#include "cfs_serializer/std_msgs_multiarraylayout_serializer.h"

int serialize_std_msgs_float64multiarray(const ::std_msgs::Float64MultiArray &send_data, unsigned char *buffer);
std_msgs::Float64MultiArray deserialize_std_msgs_float64multiarray(const unsigned char *buffer, unsigned int &length);

#endif // _std_msgs_float64multiarray_serializer_h_
