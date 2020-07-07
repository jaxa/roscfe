/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "convert/subscriber.h"
#include "define.h"
#include "convert_lib.h"

$$_SUB_INCLUDE_$$

#define SENDER_CFE 1
#define MAX_UINT8 256

unsigned char body_data[MAX_BODY_LEN];
unsigned char header_data[sizeof(RosCfeNwBridgeHeader)];
unsigned char send_no;

RosCfeNwBridgeHeader input_header_data(unsigned int body_len, unsigned short msg_id) {
  RosCfeNwBridgeHeader header;
  header.body_len = body_len;
  header.unix_time = CONVERT_RosTimeNowToSec();
  header.sender = SENDER_CFE;
  header.send_no = send_no;
  header.msg_id = msg_id;
  send_no++;
  send_no = send_no % MAX_UINT8;

  return header;
}

// Convert function name; ["topic_name:/" to "_"]_callback
$$_CALLBACK_$$

void subscriber_subscribe(void) {
$$_SUBSCRIBE_$$
}