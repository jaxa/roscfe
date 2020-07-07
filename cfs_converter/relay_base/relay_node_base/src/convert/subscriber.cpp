/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "convert/subscriber.h"

$$_SUB_CALLBACK_$$

RosCfeNwBridgeHeader Subscriber::input_header_data(uint32_t body_len, uint16_t msg_id) {
  RosCfeNwBridgeHeader header;
  header.body_len = body_len;
  header.unix_time = ros::Time::now().toSec();
  header.sender = SENDER_ROS;
  header.send_no = this->send_no_;
  header.msg_id = msg_id;
  this->send_no_++;
  this->send_no_ = this->send_no_ % MAX_UINT8;
  return header;
}

Subscriber::Subscriber(ros::NodeHandle nh, Communication *comm) : nh_(nh), comm_(comm) {}

void Subscriber::subscribe() {
$$_SUBSCRIBE_$$
}

