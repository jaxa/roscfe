/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef TWIST_STAMPED_H
#define TWIST_STAMPED_H

#include "cfe.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  geometry_msgs::Twist twist;

  void vector2pointer() {
    header.vector2pointer();
    twist.vector2pointer();
  }

  void pointer2vector() {
    header.pointer2vector();
    twist.pointer2vector();
  }

  void deleteData() {
    header.deleteData();
    twist.deleteData();
  }

  void string2pointer() {
    header.string2pointer();
    twist.string2pointer();
  }

  void pointer2string() {
    header.pointer2string();
    twist.pointer2string();
  }
} TwistStamped;

typedef TwistStamped *const TwistStampedConstPtr;
}

#endif // TWIST_STAMPED_H
