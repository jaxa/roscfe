/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

#include <sstream>

#include "convert_lib.h"
extern "C"
{
    #include "cfe.h"

    void msgCallback(void* msg);
    void HK_PS_AppMain();

    uint16 TopicNo1_;
}
