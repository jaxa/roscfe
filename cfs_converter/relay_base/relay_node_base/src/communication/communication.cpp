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
#include "relay_node/communication/communication.h"

/**
 * Constructor
 * @param nh Node handle
 */
Communication::Communication(ros::NodeHandle nh) : nh_(nh), pub_(Publisher(nh)) {
  this->socket_ = 0;
  this->socket_send_ = 0;
  this->socket_recv_ = 0;
  this->next_ring_no_ = 0;
  this->protocol_ = -1;
  this->thread_ = nullptr;
  this->is_continue_ = 1;
  this->ring_buffer_.buf = nullptr;

  this->read_property();
  this->ring_buffer_.buf = new RosCfeNwBridgeBuffer[this->ring_buffer_len_];
  if (this->ring_buffer_.buf == nullptr) {
    throw std::runtime_error("Memory could not be secured. Please lower the value of ring_length of Connection.launch");
  }
}

/**
 * Destructor
 */
Communication::~Communication() {
  if (this->ring_buffer_.buf == nullptr) {
    delete[] this->ring_buffer_.buf;
  }
  if (this->thread_ == nullptr) {
    this->is_continue_ = 0;
    delete this->thread_;
  }
}

/**
 * Read properties.
 */
void Communication::read_property() {
  std::string pkg_path = ros::package::getPath("relay_node");
  std::string launch_path = pkg_path + "/launch/Connection.launch";

  boost::property_tree::ptree pt;
  boost::property_tree::xml_parser::read_xml(launch_path.c_str(), pt);

  if (boost::optional<std::string> ip_addr = pt.get_optional<std::string>("launch.ros_relay_node.<xmlattr>.ip_addr")) {
    this->ip_addr_ = ip_addr.get();
  } else {
    throw std::runtime_error("IP address was not specified in Connection.launch");
  }
  if (boost::optional<uint32_t> port = pt.get_optional<uint32_t>("launch.ros_relay_node.<xmlattr>.port")) {
    this->port_no_ = port.get();
    this->port_no_recv_ = port.get();
  } else {
    throw std::runtime_error("Port was not specified in Connection.launch");
  }
  if (boost::optional<uint32_t> port_send = pt.get_optional<uint32_t>("launch.ros_relay_node.<xmlattr>.port_send")) {
    this->port_no_send_ = port_send.get();
  } else {
    throw std::runtime_error("Port was not specified in Connection.launch");
  }
  if (boost::optional<std::string> protocol = pt.get_optional<std::string>("launch.ros_relay_node.<xmlattr>.protocol")) {
    std::string pro = protocol.get();
    if (pro == "tcp") {
      this->protocol_ = PROTOCOL_TCP;
    } else if (pro == "udp") {
      this->protocol_ = PROTOCOL_UDP;
    } else {
      throw std::runtime_error("Protocol was not set \"tcp\" or \"udp\". ");
    }
  } else {
    throw std::runtime_error("Protocol was not specified in Connection.launch");
  }
  if (boost::optional<uint32_t> ring_buffer_len = pt.get_optional<uint32_t>("launch.ros_relay_node.<xmlattr>.ring_length")) {
    this->ring_buffer_len_ = ring_buffer_len.get();
  } else {
    throw std::runtime_error("Ring buffer length was not specified in Connection.launch");
  }
}

/**
 * Init communication.
 */
int Communication::init() {
  // ----------------------
  // Check connection
  // ----------------------
  ROS_INFO("Start up server.");
  if (this->protocol_ == PROTOCOL_TCP) {
    this->socket_ = tcp_server_init(this->ip_addr_.c_str(), this->port_no_);
  } else if (this->protocol_ == PROTOCOL_UDP) {
    this->socket_recv_ = udp_server_init(this->ip_addr_.c_str(), this->port_no_recv_);
    this->socket_send_ = udp_client_init(this->ip_addr_.c_str(), this->port_no_send_);
  }
  if (this->socket_ == -5) {
    // Abort
    throw std::runtime_error("Forced termination.");
  } else if (this->socket_ < 0) {
    throw std::runtime_error("Connection is failed(" + std::to_string(this->socket_) + ").");
  }

  // ----------------------
  // Init ring buffer
  // ----------------------
  for (size_t ii = 0; ii < this->ring_buffer_len_; ii++) {
    this->ring_buffer_.buf[ii].is_treated = TREATED;
  }

  // -----------------------------------------------------
  // Start thread
  // -----------------------------------------------------
  this->thread_ = new std::thread(&Communication::read_callback, this);
}

/**
 * Write into socket.
 * @param size    Written data size
 * @param buffer  Written data
 * @param ret     Result (0: success, -1: error)
 * @return Written data size
 */
unsigned int Communication::write(unsigned int size, unsigned char *buffer, char *ret) {
  int write_size = 0;
  if (this->protocol_ == PROTOCOL_TCP) {
    write_size = tcp_write(this->socket_, buffer, size, ret);
  } else if (this->protocol_ == PROTOCOL_UDP) {
    write_size = udp_write(this->socket_send_, buffer, size, this->ip_addr_.c_str(), this->port_no_send_, ret);
  }
  if (size > 488447517) {
    printf("wrote big data(size: %u) \n", size);
  }
  return write_size;
}

/**
 * Read from socket.
 * @param size    Read data size
 * @param buffer  Read data
 * @return Result
 */
int Communication::read(unsigned int size, unsigned char *buffer) {
  int read_result;
  if (this->protocol_ == PROTOCOL_TCP) {
    read_result = tcp_read(this->socket_, size, buffer);
  } else if (this->protocol_ == PROTOCOL_UDP) {
    read_result = udp_read(this->socket_recv_, size, buffer);
  }
  return read_result;
}

/**
 * Close socket.
 * @return Result
 */
int Communication::close() {
  int status;
  if (this->protocol_ == PROTOCOL_TCP) {
    status = tcp_close(this->socket_);
  } else if (this->protocol_ == PROTOCOL_UDP) {
    status = udp_close(this->socket_send_);
    status = udp_close(this->socket_recv_);
  }
  return status;
}

/**
 * Advertise message.
 */
void Communication::advertise() {
  this->pub_.advertise();
}

/**
 * Publish data from ring buffer.
 */
void Communication::publish_recv_data() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->pub_.read_ring_buffer(this->ring_buffer_, this->next_ring_no_, this->ring_buffer_len_);
}

/**
 * Read data from socket.
 */
void Communication::read_callback() {
  ROS_INFO("Communication::read_callback is started. ");

  uint8_t header_data[SIZEOF_HEADER];
  uint8_t body_data[MAX_BODY_LEN];
  RosCfeNwBridgeHeader header;
  int result;

  // Receive data which is sent asynchronously, and save it in ring buffer.
  while (this->is_continue_) {
    // receive header
    result = this->read(SIZEOF_HEADER, header_data);
    if (result < 0) {
      ROS_ERROR("Receive header data is failed.");
      continue;
    }

    // deserialize header
    header = deserialize_ros_cfe_header(header_data);
    for (size_t ii = 0; ii < SIZEOF_HEADER; ii++) {
      header_data[ii] = 0;
    }

    // receive body
    result = this->read(header.body_len, body_data);
    if (result) {
      ROS_ERROR("Receive body data is failed. ");
      continue;
    }
    // save to ring buffer
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->ring_buffer_.buf[this->next_ring_no_].is_treated == NOT_TREAT) {
        ROS_WARN("wrote receive data(header.msg_id: %x -> %x).", this->ring_buffer_.buf[this->next_ring_no_].header.msg_id, header.msg_id);
      }
      this->ring_buffer_.buf[this->next_ring_no_].header.body_len = header.body_len;
      this->ring_buffer_.buf[this->next_ring_no_].header.unix_time = header.unix_time;
      this->ring_buffer_.buf[this->next_ring_no_].header.sender = header.sender;
      this->ring_buffer_.buf[this->next_ring_no_].header.send_no = header.send_no;
      this->ring_buffer_.buf[this->next_ring_no_].header.msg_id = header.msg_id;
      for (size_t ii = 0; ii < header.body_len; ii++) {
        this->ring_buffer_.buf[this->next_ring_no_].body.data[ii] = body_data[ii];
      }
      this->ring_buffer_.buf[this->next_ring_no_].is_treated = NOT_TREAT;
      this->next_ring_no_++;
      this->next_ring_no_ = this->next_ring_no_ % this->ring_buffer_len_;
    }
  }
}
