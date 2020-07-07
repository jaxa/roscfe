/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_empty_serializer_h_
#define _std_msgs_empty_serializer_h_

#include "util.h"

#include "../convert/std_msgs/Empty.h"

int serialize_std_msgs_empty(const ::std_msgs::Empty &send_data, unsigned char *buffer);
std_msgs::Empty deserialize_std_msgs_empty(const unsigned char *buffer, unsigned int &length);

#endif // _std_msgs_empty_serializer_h_
