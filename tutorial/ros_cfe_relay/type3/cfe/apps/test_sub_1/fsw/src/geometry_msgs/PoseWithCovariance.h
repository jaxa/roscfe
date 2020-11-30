/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef POSE_WITH_COVARIANCE_H
#define POSE_WITH_COVARIANCE_H

#include "../geometry_msgs/Pose.h"
#include "cfe.h"

namespace geometry_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  Pose pose;
  float covariance[36];

  void vector2pointer() { pose.vector2pointer(); }

  void pointer2vector() { pose.pointer2vector(); }

  void deleteData() { pose.deleteData(); }

  void string2pointer() { pose.string2pointer(); }

  void pointer2string() { pose.pointer2string(); }
} PoseWithCovariance;
typedef PoseWithCovariance *const PoseWithCovarianceConstPtr;
}

#endif // POSE_WITH_COVARIANCE_H
