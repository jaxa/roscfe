/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _ardrone_autonomy_navdata_serializer_h_
#define _ardrone_autonomy_navdata_serializer_h_

#include "cfe.h"
#include "util.h"

#include "../convert/ardrone_autonomy/Navdata.h"
#include "cfs_serializer/std_msgs_header_serializer.h"

int serialize_ardrone_autonomy_navdata(const ::ardrone_autonomy::Navdata &send_data, unsigned char *buffer);
ardrone_autonomy::Navdata deserialize_ardrone_autonomy_navdata(const unsigned char *buffer, unsigned int &length);

#endif // _ardrone_autonomy_navdata_serializer_h_
