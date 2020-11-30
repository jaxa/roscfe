/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef NAVDATA_H
#define NAVDATA_H

#include "../std_msgs/Header.h"
#include "cfe.h"
#include <string>
#include <vector>

namespace ardrone_autonomy {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  float batteryPercent;

  uint32 state;
  int32 magX;
  int32 magY;
  int32 magZ;

  int32 pressure;

  int32 temp;

  float wind_speed;
  float wind_angle;
  float wind_comp_angle;

  float rotX;
  float rotY;
  float rotZ;

  int32 altd;

  float vx;
  float vy;
  float vz;

  float ax;
  float ay;
  float az;

  uint8 motor1;
  uint8 motor2;
  uint8 motor3;
  uint8 motor4;

  uint32 tags_count;
  std::vector<uint32> tags_type;
  std::vector<uint32> tags_xc;
  std::vector<uint32> tags_yc;
  std::vector<uint32> tags_width;
  std::vector<uint32> tags_height;
  std::vector<float> tags_orientation;
  std::vector<float> tags_distance;

  float tm;

  uint32 *tags_typeData;
  uint32 *tags_xcData;
  uint32 *tags_ycData;
  uint32 *tags_widthData;
  uint32 *tags_heightData;
  float *tags_orientationData;
  float *tags_distanceData;
  uint32 tags_typeDataSize;
  uint32 tags_xcDataSize;
  uint32 tags_ycDataSize;
  uint32 tags_widthDataSize;
  uint32 tags_heightDataSize;
  uint32 tags_orientationDataSize;
  uint32 tags_distanceDataSize;

  void vector2pointer() {
    tags_typeData = (uint32 *)malloc(sizeof(uint32) * tags_type.size());
    for (size_t ii = 0; ii < tags_type.size(); ii++) {
      tags_typeData[ii] = tags_type[ii];
    }
    tags_typeDataSize = tags_type.size();
    std::vector<uint32>().swap(tags_type);

    tags_xcData = (uint32 *)malloc(sizeof(uint32) * tags_xc.size());
    for (size_t ii = 0; ii < tags_xc.size(); ii++) {
      tags_xcData[ii] = tags_xc[ii];
    }
    tags_xcDataSize = tags_xc.size();
    std::vector<uint32>().swap(tags_xc);

    tags_ycData = (uint32 *)malloc(sizeof(uint32) * tags_yc.size());
    for (size_t ii = 0; ii < tags_yc.size(); ii++) {
      tags_ycData[ii] = tags_yc[ii];
    }
    tags_ycDataSize = tags_yc.size();
    std::vector<uint32>().swap(tags_yc);

    tags_widthData = (uint32 *)malloc(sizeof(uint32) * tags_width.size());
    for (size_t ii = 0; ii < tags_width.size(); ii++) {
      tags_widthData[ii] = tags_width[ii];
    }
    tags_widthDataSize = tags_width.size();
    std::vector<uint32>().swap(tags_width);

    tags_heightData = (uint32 *)malloc(sizeof(uint32) * tags_height.size());
    for (size_t ii = 0; ii < tags_height.size(); ii++) {
      tags_heightData[ii] = tags_height[ii];
    }
    tags_heightDataSize = tags_height.size();
    std::vector<uint32>().swap(tags_height);

    tags_orientationData =
        (float *)malloc(sizeof(float) * tags_orientation.size());
    for (size_t ii = 0; ii < tags_orientation.size(); ii++) {
      tags_orientationData[ii] = tags_orientation[ii];
    }
    tags_orientationDataSize = tags_orientation.size();
    std::vector<float>().swap(tags_orientation);

    tags_distanceData = (float *)malloc(sizeof(float) * tags_distance.size());
    for (size_t ii = 0; ii < tags_distance.size(); ii++) {
      tags_distanceData[ii] = tags_distance[ii];
    }
    tags_distanceDataSize = tags_distance.size();
    std::vector<float>().swap(tags_distance);
  }

  void pointer2vector() {
    uint32 tags_type_size = tags_typeDataSize;
    std::vector<uint32>().swap(tags_type);
    tags_type = std::vector<uint32>();
    for (size_t ii = 0; ii < tags_type_size; ii++) {
      tags_type.push_back(tags_typeData[ii]);
    }

    uint32 tags_xc_size = tags_xcDataSize;
    std::vector<uint32>().swap(tags_xc);
    tags_xc = std::vector<uint32>();
    for (size_t ii = 0; ii < tags_xc_size; ii++) {
      tags_xc.push_back(tags_xcData[ii]);
    }

    uint32 tags_yc_size = tags_ycDataSize;
    std::vector<uint32>().swap(tags_yc);
    tags_yc = std::vector<uint32>();
    for (size_t ii = 0; ii < tags_yc_size; ii++) {
      tags_yc.push_back(tags_ycData[ii]);
    }

    uint32 tags_width_size = tags_widthDataSize;
    std::vector<uint32>().swap(tags_width);
    tags_width = std::vector<uint32>();
    for (size_t ii = 0; ii < tags_width_size; ii++) {
      tags_width.push_back(tags_widthData[ii]);
    }

    uint32 tags_height_size = tags_heightDataSize;
    std::vector<uint32>().swap(tags_height);
    tags_height = std::vector<uint32>();
    for (size_t ii = 0; ii < tags_height_size; ii++) {
      tags_height.push_back(tags_heightData[ii]);
    }

    uint32 tags_orientation_size = tags_orientationDataSize;
    std::vector<float>().swap(tags_orientation);
    tags_orientation = std::vector<float>();
    for (size_t ii = 0; ii < tags_orientation_size; ii++) {
      tags_orientation.push_back(tags_orientationData[ii]);
    }

    uint32 tags_distance_size = tags_distanceDataSize;
    std::vector<float>().swap(tags_distance);
    tags_distance = std::vector<float>();
    for (size_t ii = 0; ii < tags_distance_size; ii++) {
      tags_distance.push_back(tags_distanceData[ii]);
    }
  }

  void deleteData() {
    free(tags_typeData);
    std::vector<uint32>().swap(tags_type);
    free(tags_xcData);
    std::vector<uint32>().swap(tags_xc);
    free(tags_ycData);
    std::vector<uint32>().swap(tags_yc);
    free(tags_widthData);
    std::vector<uint32>().swap(tags_width);
    free(tags_heightData);
    std::vector<uint32>().swap(tags_height);
    free(tags_orientationData);
    std::vector<float>().swap(tags_orientation);
    free(tags_distanceData);
    std::vector<float>().swap(tags_distance);
  }

  void string2pointer() { header.string2pointer(); }

  void pointer2string() { header.pointer2string(); }
} Navdata;
typedef Navdata *const NavdataConstPtr;
} // namespace ardrone_autonomy
#endif // CFS_CONVERTER_PORTING_FILES_ARDRONE_AUTONOMY_NAVDATA_H_
