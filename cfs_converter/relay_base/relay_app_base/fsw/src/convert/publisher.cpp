/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "convert/publisher.h"
#include "convert_lib.h"

$$_PUB_INCLUDE_$$

$$_TOPIC_NO_DEFINE_$$

unsigned int loop_rate;

void publisher_advertise() {
  int executeRate = 1000;
  loop_rate = CONVERT_RosRate(executeRate);

$$_ADVERTISE_$$
}

void publisher_read_ring_buffer(RosCfeNwBridgeRingBuffer &ring_buffer, unsigned int next_ring_no) {
  // Process data to be written next.
  unsigned int ring_no = next_ring_no;
  unsigned int ii;

  // Process all received data.
  for (ii = 0; ii < RING_BUF_LEN; ii++) {
    if (ring_buffer.buf[ring_no].is_treated == NOT_TREAT) {
      // Process unprocessed data.
      publisher_publish(ring_buffer.buf[ring_no].header, ring_buffer.buf[ring_no].body);

      // Set processed flag.
      ring_buffer.buf[ring_no].is_treated = TREATED;
    }
    ring_no = (ring_no + 1) % RING_BUF_LEN;
  }
}

void publisher_rate_sleep(void) {
  CONVERT_RosRateSleep(loop_rate, relay_pub_topic_no001);
}

void publisher_publish(const RosCfeNwBridgeHeader &header, const RosCfeNwBridgeBody &body) {
  unsigned int length;
  unsigned short msg_id = header.msg_id;

  // Check message ID.
$$_PUBLISH_$$
}

