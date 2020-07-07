/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "cfs_serializer/std_msgs_empty_serializer.h"

int serialize_std_msgs_empty(const ::std_msgs::Empty &send_data, unsigned char *buffer) {
  unsigned int len = 0;

  return len;
}

std_msgs::Empty deserialize_std_msgs_empty(const unsigned char *buffer, unsigned int &length) {
  std_msgs::Empty msg;
  unsigned int ii;
  unsigned int len;
  length = 0;

  return msg;
}
