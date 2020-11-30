/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef WRENCH_STAMPED_H
#define WRENCH_STAMPED_H

#include "cfe.h"

#include "../geometry_msgs/Wrench.h"
#include "../std_msgs/Header.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  Wrench wrench;

  void vector2pointer() {
    header.vector2pointer();
    wrench.vector2pointer();
  }

  void pointer2vector() {
    header.pointer2vector();
    wrench.pointer2vector();
  }

  void deleteData() {
    header.deleteData();
    wrench.deleteData();
  }

  void string2pointer() {
    header.string2pointer();
    wrench.string2pointer();
  }

  void pointer2string() {
    header.pointer2string();
    wrench.pointer2string();
  }
} WrenchStamped;
typedef WrenchStamped *const WrenchStampedConstPtr;
}

#endif // WRENCH_STAMPED_H
