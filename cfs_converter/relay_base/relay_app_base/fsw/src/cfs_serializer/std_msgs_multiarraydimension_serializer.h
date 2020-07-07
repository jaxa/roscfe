/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_multiarraydimension_serializer_h_
#define _std_msgs_multiarraydimension_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/std_msgs/MultiArrayDimension.h"

int serialize_std_msgs_multiarraydimension(const ::std_msgs::MultiArrayDimension &send_data, unsigned char *buffer);
std_msgs::MultiArrayDimension deserialize_std_msgs_multiarraydimension(const unsigned char *buffer, unsigned int &length);

#endif // _std_msgs_multiarraydimension_serializer_h_
