/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef EMPTY_H
#define EMPTY_H

#include "cfe.h"

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];

  void vector2pointer() {}

  void pointer2vector() {}

  void deleteData() {}

  void string2pointer() {}

  void pointer2string() {}
} Empty;
typedef Empty *const EmptyConstPtr;
}
#endif // EMPTY_H
