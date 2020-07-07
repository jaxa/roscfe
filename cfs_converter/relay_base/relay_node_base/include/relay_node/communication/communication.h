/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _Communication_h_
#define _Communication_h_

#include "convert/publisher.h"
#include "relay_node/define.h"

#include "pthread.h"
#include "ros/ros.h"
#include <exception>
#include <mutex>
#include <ros/package.h>
#include <thread>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

extern "C" {
int8_t tcp_server_init(const char *ip_addr, uint16_t port);
int8_t tcp_read(int8_t dst_sock, uint32_t buf_size, uint8_t *read_data);
uint32_t tcp_write(int8_t dst_sock, uint8_t *data, uint32_t data_size,
                   char *ret);
int8_t tcp_close(int8_t dst_sock);
int8_t udp_server_init(const char *ip_addr, uint16_t port);
int8_t udp_client_init(const char *ip_addr, uint16_t port);
int8_t udp_read(int8_t dst_sock, uint32_t buf_size, uint8_t *read_data);
uint32_t udp_write(int8_t dst_sock, uint8_t *write_data, uint32_t data_size,
                   const char *ipaddr, uint16_t port, char *ret);
int8_t udp_close(int8_t dst_sock);
}

// definition
#define PROTOCOL_TCP 0
#define PROTOCOL_UDP 1

class Communication {
public:
  /**
   * Constructor
   * @param nh NodeHandle
   */
  Communication(ros::NodeHandle nh);

  /**
   * Destructor
   */
  ~Communication();

  /**
   * Init socket.
   * @return Result
   */
  int init();

  /**
   * Write into socket.
   * @param size    Written data size
   * @param buffer  Written data
   * @param ret     Result (0: success, -1: error)
   * @return Written data size
   */
  unsigned int write(unsigned int size, unsigned char *buffer, char *ret);

  /**
   * Read from socket.
   * @param size    Read data size
   * @param buffer  Read data
   * @return Result
   */
  int read(unsigned int size, unsigned char *buffer);

  /**
   * Close socket.
   * @return Result
   */
  int close();

  /**
   * Advertise message.
   */
  void advertise();

  /**
   * Publish data from ring buffer.
   */
  void publish_recv_data();

private:
  /**
   * Read config from launch-file.
   */
  void read_property();

  /**
   * Read data from socket.
   */
  void read_callback();

  // Protocol
  uint8_t protocol_;
  // TCP socket
  int socket_;
  // UDP socket (used for sending)
  int socket_send_;
  // UDP socket (used for receiving)
  int socket_recv_;
  // IP address
  std::string ip_addr_;
  // TCP port number
  uint32_t port_no_;
  // UDP port number (used for sending)
  uint32_t port_no_send_;
  // UDP port number (used for receiving)
  uint32_t port_no_recv_;

  // Ring buffer
  RosCfeNwBridgeRingBuffer ring_buffer_;
  // Length of ring buffer
  uint32_t ring_buffer_len_;
  // Next ID to be written in ring buffer
  uint32_t next_ring_no_;

  // NodeHandle
  ros::NodeHandle nh_;
  // Publisher
  Publisher pub_;

  // Thread
  std::thread *thread_;
  // Mutex
  std::mutex mutex_;
  // Whether read_callback continues to receive or not.
  uint8_t is_continue_;
};

#endif // _Communication_h_
