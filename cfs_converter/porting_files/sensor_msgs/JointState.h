/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "../std_msgs/Header.h"
#include "cfe.h"
#include <string>
#include <vector>

namespace sensor_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;

  double *positionData;
  double *velocityData;
  double *effortData;
  uint32 positionDataSize;
  uint32 velocityDataSize;
  uint32 effortDataSize;

  void vector2pointer() {
    positionData = (double *)malloc(sizeof(double) * position.size());
    for (size_t ii = 0; ii < position.size(); ii++) {
      positionData[ii] = position[ii];
    }
    positionDataSize = position.size();
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(position);

    velocityData = (double *)malloc(sizeof(double) * velocity.size());
    for (size_t ii = 0; ii < velocity.size(); ii++) {
      velocityData[ii] = velocity[ii];
    }
    velocityDataSize = velocity.size();
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(velocity);

    effortData = (double *)malloc(sizeof(double) * effort.size());
    for (size_t ii = 0; ii < effort.size(); ii++) {
      effortData[ii] = effort[ii];
    }
    effortDataSize = effort.size();
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(effort);
  }

  void pointer2vector() {
    uint32 position_size = positionDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(position);
    // Unless the above release is performed, memory cannot be accessed
    position = std::vector<double>();
    for (size_t ii = 0; ii < position_size; ii++) {
      position.push_back(positionData[ii]);
    }
    uint32 velocity_size = velocityDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(velocity);
    // Unless the above release is performed, memory cannot be accessed
    velocity = std::vector<double>();
    for (size_t ii = 0; ii < velocity_size; ii++) {
      velocity.push_back(velocityData[ii]);
    }
    uint32 effort_size = effortDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<double>().swap(effort);
    // Unless the above release is performed, memory cannot be accessed
    effort = std::vector<double>();
    for (size_t ii = 0; ii < effort_size; ii++) {
      effort.push_back(effortData[ii]);
    }
  }

  void deleteData() {
    free(positionData);
    std::vector<double>().swap(position);
    free(velocityData);
    std::vector<double>().swap(velocity);
    free(effortData);
    std::vector<double>().swap(effort);
  }

  void string2pointer() { header.string2pointer(); }

  void pointer2string() { header.pointer2string(); }
} JointState;
typedef JointState *const JointStateConstPtr;
}

#endif // JOINT_STATE_H
