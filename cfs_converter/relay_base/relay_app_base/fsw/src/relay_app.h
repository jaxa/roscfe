/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
/*************************************************************************
** Includes
*************************************************************************/
#ifndef _relay_app_h_
#define _relay_app_h_
#include "convert_lib.h"
#include "util.h"
#include "convert/subscriber.h"

extern "C"
{
  #include "cfe.h"

  /**
   * cfe_relay_app main
   */
  void RELAY_AppMain(void);
}
#endif /* _relay_app_h_ */
