/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef POINT_H
#define POINT_H

#include "cfe.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  float x;
  float y;
  float z;

  void vector2pointer() {}

  void pointer2vector() {}

  void deleteData() {}

  void string2pointer() {}

  void pointer2string() {}
} Point;
typedef Point *const PointConstPtr;
} // namespace geometry_msgs

#endif // CFS_CONVERTER_PORTING_FILES_GEOMETRY_MSGS_POINT_H_
