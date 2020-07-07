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
#include <relay_app.h>

#include <string>

/**
 * Main
 */
void RELAY_AppMain(void)
{
  OS_printf("RELAY_AppMain \n");

  CONVERT_RosInit(0, NULL);

  // Start connection
  communication_init();

  // publish setting
  communication_advertise();

  // subscribe setting
  subscriber_subscribe();

  while (CONVERT_RosOk()) {
    // Processing Subscribe data
    CONVERT_RosSpinOnce();

    // Retrieve and publish received data
    communication_publish_recv_data();
  }
  communication_close();
}