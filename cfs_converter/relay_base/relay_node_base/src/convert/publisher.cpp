/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "convert/publisher.h"

$$_PUB_INCLUDE_$$

Publisher::Publisher(ros::NodeHandle nh) : nh_(nh) {}

void Publisher::advertise() {
$$_ADVERTISE_$$
}

void Publisher::read_ring_buffer(RosCfeNwBridgeRingBuffer &ring_buffer, uint32_t next_ring_no, uint32_t ring_buffer_len) {
  uint32_t ring_no = next_ring_no;
  // Repeat processing until all data is processed.
  for (size_t ii = 0; ii < ring_buffer_len; ii++) {
    if (ring_buffer.buf[ring_no].is_treated == NOT_TREAT) {
      // Process unprocessed data
      this->publish(ring_buffer.buf[ring_no].header, ring_buffer.buf[ring_no].body);
      // Set processed flag
      ring_buffer.buf[ring_no].is_treated = TREATED;
    }
    ring_no = (ring_no + 1) % ring_buffer_len;
  }
}

void Publisher::publish(RosCfeNwBridgeHeader header, RosCfeNwBridgeBody body) {
  uint32_t length;
  uint16_t msg_id = header.msg_id;

$$_PUBLISH_$$
}
