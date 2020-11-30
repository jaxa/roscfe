/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef WRENCH_H
#define WRENCH_H

#include "cfe.h"

#include "geometry_msgs/Vector3.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  Vector3 force;
  Vector3 torque;

  void vector2pointer() {
    force.vector2pointer();
    torque.vector2pointer();
  }

  void pointer2vector() {
    force.pointer2vector();
    torque.pointer2vector();
  }

  void deleteData() {
    force.deleteData();
    torque.deleteData();
  }

  void string2pointer() {
    force.string2pointer();
    torque.string2pointer();
  }

  void pointer2string() {
    force.pointer2string();
    torque.pointer2string();
  }
} Wrench;
typedef Wrench *const WrenchConstPtr;
}

#endif // WRENCH_H
