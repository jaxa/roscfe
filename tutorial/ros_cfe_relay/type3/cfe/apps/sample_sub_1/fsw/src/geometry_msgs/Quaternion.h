/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef GEOMETRY_MSGS_QUATERNION_H
#define GEOMETRY_MSGS_QUATERNION_H

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  float x;
  float y;
  float z;
  float w;

  void vector2pointer() {}

  void pointer2vector() {}

  void deleteData() {}

  void string2pointer() {}

  void pointer2string() {}
} Quaternion;
typedef Quaternion *const QuaternionConstPtr;
} // namespace geometry_msgs

#endif // GEOMETRY_MSGS_QUATERNION_H