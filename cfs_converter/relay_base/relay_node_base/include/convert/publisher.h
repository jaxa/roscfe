/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _Publisher_h_
#define _Publisher_h_

#include "relay_node/define.h"
#include "ros/ros.h"

class Publisher {
public:
  /**
   * Constructor
   */
  Publisher(ros::NodeHandle nh);

  /**
   * Advertise the message to be published.
   */
  void advertise();

  /**
   * Read the ring buffer and publish data.
   * @param ring_buffer     Ring buffer
   * @param next_ring_no    ID to be read
   * @param ring_buffer_len Length of the ring buffer
   */
  void read_ring_buffer(RosCfeNwBridgeRingBuffer &ring_buffer,
                        uint32_t next_ring_no, uint32_t ring_buffer_len);

private:
  /**
   * Publish data.
   * @param header Message header
   * @param body   Message body
   */
  void publish(RosCfeNwBridgeHeader header, RosCfeNwBridgeBody body);
  ros::NodeHandle nh_;

  $$_PUBLISHER_COUNT_$$
};

#endif // _Publisher_h_