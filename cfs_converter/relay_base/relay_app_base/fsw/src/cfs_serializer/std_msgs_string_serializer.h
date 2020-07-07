/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _std_msgs_string_serializer_h_
#define _std_msgs_string_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/std_msgs/String.h"

int serialize_std_msgs_string(const ::std_msgs::String &send_data, unsigned char *buffer);
std_msgs::String deserialize_std_msgs_string(const unsigned char *buffer, unsigned int &length);

#endif // _std_msgs_string_serializer_h_
