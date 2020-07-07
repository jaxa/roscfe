/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _Subscriber_h_
#define _Subscriber_h_

#include "relay_node/communication/communication.h"
#include "relay_node/define.h"
#include "ros/ros.h"

#define SENDER_ROS 0
#define MAX_UINT8 256

$$_SUB_INCLUDE_$$

class Subscriber {
public:
  /**
   * constructor
   * @param nh NodeHandle
   * @param comm Communication class instance
   */
  Subscriber(ros::NodeHandle nh, Communication *comm);

  /**
   * Define a subscription callback function
   */
  void subscribe();

  $$_SUBSCRIBE_CALLBACK_$$

private:
  // NodeHandle
  ros::NodeHandle nh_;
  // Communication class instance
  Communication *comm_;
  // Body buffer
  uint8_t body_data_[MAX_BODY_LEN];
  // Header buffer
  uint8_t header_data_[sizeof(RosCfeNwBridgeHeader)];
  // Send number
  uint8_t send_no_;

  /**
   * Create communication header from body length and message ID
   * @param body_len body length
   * @param msg_id message ID
   * @return communication header
   */
  RosCfeNwBridgeHeader input_header_data(uint32_t body_len, uint16_t msg_id);

  $$_SUBSCRIBER_COUNT_$$
};

#endif // _Subscriber_h_