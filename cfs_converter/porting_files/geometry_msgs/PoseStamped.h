/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef POSE_STAMPED_H
#define POSE_STAMPED_H

#include "cfe.h"

#include "../geometry_msgs/Pose.h"
#include "../std_msgs/Header.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  Pose pose;

  void vector2pointer() {
    header.vector2pointer();
    pose.vector2pointer();
  }

  void pointer2vector() {
    header.pointer2vector();
    pose.pointer2vector();
  }

  void deleteData() {
    header.deleteData();
    pose.deleteData();
  }

  void string2pointer() {
    header.string2pointer();
    pose.string2pointer();
  }

  void pointer2string() {
    header.pointer2string();
    pose.pointer2string();
  }
} PoseStamped;
typedef PoseStamped *const PoseStampedConstPtr;
} // namespace geometry_msgs

#endif // CFS_CONVERTER_PORTING_FILES_GEOMETRY_MSGS_POSESTAMPED_H_
