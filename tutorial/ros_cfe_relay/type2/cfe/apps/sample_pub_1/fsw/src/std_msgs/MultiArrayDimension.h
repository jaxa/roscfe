/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef MULTI_ARRAY_DIMENSION_H
#define MULTI_ARRAY_DIMENSION_H

#include "cfe.h"

#include <string>

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std::string label;
  uint size;
  uint stride;

  void vector2pointer() {}

  void pointer2vector() {}

  void deleteData() {}

  void string2pointer() {}

  void pointer2string() {}
} MultiArrayDimension;
typedef MultiArrayDimension *const MultiArrayDimensionConstPtr;
}

#endif // MULTI_ARRAY_DIMENSION_H
