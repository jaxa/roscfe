/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef FLOAT_64_MULTI_ARRAY_H
#define FLOAT_64_MULTI_ARRAY_H

#include "cfe.h"

#include "MultiArrayLayout.h"
#include <vector>

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  MultiArrayLayout layout;
  // float data[62];
  std::vector<float> data;
  float *dataData;
  uint32 dataDataSize;

  void vector2pointer() {
    layout.vector2pointer();
    dataData = (float *)malloc(data.size() * sizeof(float));
    for (size_t ii = 0; ii < data.size(); ii++) {
      dataData[ii] = data[ii];
    }
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<float>().swap(data);
  }

  void pointer2vector() {
    layout.pointer2vector();
    uint32 data_size = dataDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<float>().swap(data);
    // Unless the above release is performed, memory cannot be accessed
    data = std::vector<float>();
    for (size_t ii = 0; ii < data_size; ii++) {
      data.push_back(dataData[ii]);
    }
  }

  void deleteData() {
    layout.deleteData();
    free(dataData);
    std::vector<float>().swap(data);
  }

  void string2pointer() { layout.string2pointer(); }

  void pointer2string() { layout.pointer2string(); }
} Float64MultiArray;
typedef Float64MultiArray *const Float64MultiArrayConstPtr;
}

#endif // FLOAT_64_MULTI_ARRAY_H
