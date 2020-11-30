/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef MULTI_ARRAY_LAYOUT_H
#define MULTI_ARRAY_LAYOUT_H

#include "MultiArrayDimension.h"
#include "cfe.h"
#include <vector>

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  // MultiArrayDimension dim[];
  // MultiArrayDimension* dim;
  std::vector<MultiArrayDimension> dim;
  uint data_offset;
  MultiArrayDimension *dimData;
  uint32 dimDataSize;

  void vector2pointer() {
    dimData =
        (MultiArrayDimension *)malloc(dim.size() * sizeof(MultiArrayDimension));
    for (size_t ii = 0; ii < dim.size(); ii++) {
      dimData[ii] = dim[ii];
    }
    dimDataSize = dim.size();
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<MultiArrayDimension>().swap(dim);
  }

  void pointer2vector() {
    uint32 dim_size = dimDataSize;
    // The memory of std::vector is released by
    // swapping the temporary object
    std::vector<MultiArrayDimension>().swap(dim);
    // Unless the above release is performed, memory cannot be accessed
    dim = std::vector<MultiArrayDimension>();
    for (size_t ii = 0; ii < dim_size; ii++) {
      dim.push_back(dimData[ii]);
    }
  }

  void deleteData() {
    free(dimData);
    std::vector<MultiArrayDimension>().swap(dim);
  }

  void string2pointer() {}

  void pointer2string() {}
} MultiArrayLayout;
typedef MultiArrayLayout *const MultiArrayLayoutConstPtr;
}

#endif // MULTI_ARRAY_LAYOUT_H
