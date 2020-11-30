/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef HEADER_H
#define HEADER_H

#include "cfe.h"

#include "RosTime.h"
#include <string>

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  uint seq;
  RosTime stamp;
  std::string frame_id;

  void vector2pointer() { stamp.vector2pointer(); }

  void pointer2vector() {}

  void deleteData() { stamp.deleteData(); }

  void string2pointer() {}

  void pointer2string() {}
} Header;
typedef Header *const HeaderConstPtr;
}

#endif // HEADER_H
