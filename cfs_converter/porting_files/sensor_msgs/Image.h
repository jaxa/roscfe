/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef IMAGE_H
#define IMAGE_H

#include "../std_msgs/Header.h"
#include "cfe.h"
#include <string>
#include <vector>

namespace sensor_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std_msgs::Header header;
  uint32 height;
  uint32 width;
  std::string encoding;
  uint8 is_bigendian;
  uint32 step;
  std::vector<uint8> data;

  uint8 *dataData;
  uint32 dataDataSize;

  void vector2pointer() {
    dataData = (uint8 *)malloc(sizeof(uint8) * data.size());
    for (size_t ii = 0; ii < data.size(); ii++) {
      dataData[ii] = data[ii];
    }
    dataDataSize = data.size();
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<uint8>().swap(data);
    header.vector2pointer();
  }

  void pointer2vector() {
    uint32 data_size = dataDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<uint8>().swap(data);
    // Unless the above release is performed, memory cannot be accessed
    data = std::vector<uint8>();
    for (size_t ii = 0; ii < data_size; ii++) {
      data.push_back(dataData[ii]);
    }
    header.pointer2vector();
  }

  void deleteData() {
    free(dataData);
    std::vector<uint8>().swap(data);
    header.deleteData();
  }

  void string2pointer() { header.string2pointer(); }

  void pointer2string() { header.pointer2string(); }
} Image;

typedef Image *const ImageConstPtr;
}

#endif // IMAGE_H
