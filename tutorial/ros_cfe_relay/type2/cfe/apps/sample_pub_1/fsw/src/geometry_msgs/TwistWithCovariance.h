/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef TWIST_WITH_COVARIANCE_H
#define TWIST_WITH_COVARIANCE_H

#include "../geometry_msgs/Twist.h"
#include "cfe.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  Twist twist;
  float covariance[36];

  void vector2pointer() { twist.vector2pointer(); }

  void pointer2vector() { twist.pointer2vector(); }

  void deleteData() { twist.deleteData(); }

  void string2pointer() { twist.string2pointer(); }

  void pointer2string() { twist.pointer2string(); }
} TwistWithCovariance;
typedef TwistWithCovariance *const TwistWithCovarianceConstPtr;
}

#endif // TWIST_WITH_COVARIANCE_H
