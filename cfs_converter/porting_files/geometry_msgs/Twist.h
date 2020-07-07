/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef TWIST_H
#define TWIST_H

#include "cfe.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  Vector3 linear;
  Vector3 angular;

  void vector2pointer() {
    linear.vector2pointer();
    angular.vector2pointer();
  }

  void pointer2vector() {
    linear.pointer2vector();
    angular.pointer2vector();
  }

  void deleteData() {
    linear.deleteData();
    angular.deleteData();
  }

  void string2pointer() {
    linear.string2pointer();
    angular.string2pointer();
  }

  void pointer2string() {
    linear.pointer2string();
    angular.pointer2string();
  }
} Twist;

typedef Twist *const TwistConstPtr;
}

#endif // TWIST_H
