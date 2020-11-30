/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef POSE_H
#define POSE_H

#include "cfe.h"

#include "../geometry_msgs/Point.h"
#include "../geometry_msgs/Quaternion.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  Point position;
  Quaternion orientation;

  void vector2pointer() {
    position.vector2pointer();
    orientation.vector2pointer();
  }

  void pointer2vector() {
    position.pointer2vector();
    orientation.pointer2vector();
  }

  void deleteData() {
    position.deleteData();
    orientation.deleteData();
  }

  void string2pointer() {
    position.string2pointer();
    orientation.string2pointer();
  }

  void pointer2string() {
    position.pointer2string();
    orientation.pointer2string();
  }
} Pose;
typedef Pose *const PoseConstPtr;
} // namespace geometry_msgs

#endif // CFS_CONVERTER_PORTING_FILES_GEOMETRY_MSGS_POSE_H_
