/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef STRING_H
#define STRING_H

#include "cfe.h"
#include <string>

namespace std_msgs {
typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  std::string data;
  char *dataData;
  uint32 dataDataSize;

  void vector2pointer() {}

  void pointer2vector() {}

  void deleteData() {}

  void string2pointer() {
    dataDataSize = data.length();
    dataData = (char *)malloc(sizeof(char) * dataDataSize + 1);
    for (size_t ii = 0; ii < dataDataSize; ii++) {
      dataData[ii] = data[ii];
    }
    dataData[dataDataSize] = 0;
    std::string temp;
    temp.swap(data);
  }

  void pointer2string() {
    std::string *temp = new std::string();
    temp->swap(data);
    data = std::string(dataData);
  }
} String;

typedef String *const StringConstPtr;
}
#endif // STRING_H
