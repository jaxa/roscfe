/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "cfe.h"
// #include <vector>
#include "../geometry_msgs/PoseWithCovariance.h"
#include "../geometry_msgs/TwistWithCovariance.h"
#include "../std_msgs/Header.h"
#include <string>

namespace nav_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  std::string child_frame_id;
  geometry_msgs::PoseWithCovariance pose;
  geometry_msgs::TwistWithCovariance twist;

  void vector2pointer() {
    header.vector2pointer();
    pose.vector2pointer();
    twist.vector2pointer();
  }

  void pointer2vector() {
    header.pointer2vector();
    pose.pointer2vector();
    twist.pointer2vector();
  }

  void deleteData() {
    header.deleteData();
    pose.deleteData();
    twist.deleteData();
  }

  void string2pointer() {
    header.string2pointer();
    pose.string2pointer();
    twist.string2pointer();
  }

  void pointer2string() {
    header.pointer2string();
    pose.pointer2string();
    twist.pointer2string();
  }
} Odometry;
typedef Odometry *const OdometryConstPtr;
}

#endif // ODOMETRY_H