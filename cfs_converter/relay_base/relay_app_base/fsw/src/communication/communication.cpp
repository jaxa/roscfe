/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "communication/communication.h"

#include "define.h"
#include "pthread.h"
#include "convert/publisher.h"
#include "errno.h"

extern "C" {
  #include "communication/tcp.h"
  #include "communication/udp.h"
}

// TCP socket
int com_socket = 0;
// UDP socket (used for sending)
int com_socket_send = 0;
// UDP socket (used for receiving)
int com_socket_recv = 0;
// Protocol
int com_protocol = PROTOCOL_TCP;
// Ring buffer
RosCfeNwBridgeRingBuffer com_ring_buffer;
// Mutex
pthread_mutex_t com_mutex;
// Next id to be written into ring buffer
unsigned int com_next_ring_no = 0;

int communication_init(void) {
  int ii;

  // ----------------------
  // Check connection
  // ----------------------
  // Try connecting with TCP
  com_socket = tcp_client_init(IP_ADDR, PORT_NO);
  if (com_socket >= 0) {
    // If connection succeeds, set using protocol to TCP.
    com_protocol = PROTOCOL_TCP;
    OS_printf("Connect with TCP. \n");
  } else {
    OS_printf("com_socket: %d, \n", com_socket);
    // If connection fails, try connecting with UDP.
    com_socket_send = udp_client_init(IP_ADDR, PORT_NO_SEND);
    com_socket_recv = udp_server_init(IP_ADDR, PORT_NO_RECV);
    if (com_socket_send == COM_ERROR || com_socket_recv == COM_ERROR) {
      // If connection with neither TCP nor UDP succeed, return error.
      OS_printf("Connection is failed. \n");
      return COM_ERROR;
    }
    // If connection succeeds, set using protocol to UDP.
    com_protocol = PROTOCOL_UDP;
    OS_printf("Connect with UDP. \n");
  }

  // ----------------------
  // Init ring buffer.
  // ----------------------
  for (ii = 0; ii < RING_BUF_LEN; ii++) {
    com_ring_buffer.buf[ii].is_treated = TREATED;
  }

  // ----------------------
  // Create subscription thread.
  // ----------------------
  pthread_mutex_init(&com_mutex, NULL);
  pthread_t pthread;
  pthread_create(&pthread, NULL, &communication_read_callback, NULL);

  return 0;
}

unsigned int communication_write(unsigned int size, unsigned char *buffer, char *ret) {
  int write_size = 0;
  if (com_protocol == PROTOCOL_TCP) {
    write_size = tcp_write(com_socket, buffer, size, ret);
  } else {
    write_size = udp_write(com_socket_send, buffer, size, IP_ADDR, PORT_NO_SEND, ret);
  }
  return write_size;
}

int communication_read(unsigned int size, unsigned char* buffer) {
  int read_result = 0;
  errno = 0;
  if (com_protocol == PROTOCOL_TCP) {
    read_result = tcp_read(com_socket, size, buffer);
  } else {
    read_result = udp_read(com_socket_recv, size, buffer);
  }
  if (read_result == -1) {
    printf("error occurred(errno: %u, size: %u).\n", errno, size);
  }
  return read_result;
}

int communication_close(void) {
  int result = 0;
  if (com_protocol == PROTOCOL_TCP) {
    result = tcp_close(com_socket);
  } else {
    result = udp_close(com_socket_send);
    result = udp_close(com_socket_recv);
  }
  return result;
}

void communication_advertise(void) {
  publisher_advertise();
}

void communication_publish_recv_data(void) {
  pthread_mutex_lock(&com_mutex);
  publisher_read_ring_buffer(com_ring_buffer, com_next_ring_no);
  pthread_mutex_unlock(&com_mutex);
  publisher_rate_sleep();
}

void* communication_read_callback(void* argv) {
  OS_printf("communication_read_callback. \n");

  unsigned char *header_data = new unsigned char[SIZEOF_HEADER];
  unsigned char *body_data = new unsigned char[MAX_BODY_LEN];
  int read_size = 0;
  RosCfeNwBridgeHeader header;
  unsigned int loop_cnt;

  while (1) {
    // read header
    read_size = communication_read(SIZEOF_HEADER, header_data);
    if (read_size < 0) {
      OS_printf("read header is failed. \n");
      continue;
    }

    // deserialize to Header_t from uint8_t*
    header = deserialize_ros_cfe_header(header_data);
    for (loop_cnt = 0; loop_cnt < SIZEOF_HEADER; loop_cnt++) {
      header_data[loop_cnt] = 0;
    }

    // read body
    errno = 0;
    read_size = communication_read(header.body_len, body_data);
    if (read_size < 0) {
      OS_printf("read body is failed(errno: %d). \n", errno);
      continue;
    }

    // lock
    pthread_mutex_lock(&com_mutex);
    // save to com_ring_buffer
    com_ring_buffer.buf[com_next_ring_no].header.body_len = header.body_len;
    com_ring_buffer.buf[com_next_ring_no].header.unix_time = header.unix_time;
    com_ring_buffer.buf[com_next_ring_no].header.sender = header.sender;
    com_ring_buffer.buf[com_next_ring_no].header.send_no = header.send_no;
    com_ring_buffer.buf[com_next_ring_no].header.msg_id = header.msg_id;

    for (loop_cnt = 0; loop_cnt < header.body_len; loop_cnt++) {
      com_ring_buffer.buf[com_next_ring_no].body.data[loop_cnt] = body_data[loop_cnt];
    }
    if (com_ring_buffer.buf[com_next_ring_no].is_treated = NOT_TREAT) {
      OS_printf("rewrite data(send_no: %u, msg_id: %04x)\n", header.send_no, header.msg_id);
    }
    com_ring_buffer.buf[com_next_ring_no].is_treated = NOT_TREAT;
    com_next_ring_no++;
    com_next_ring_no = com_next_ring_no % RING_BUF_LEN;
    // unlock
    pthread_mutex_unlock(&com_mutex);
  }
  delete[] header_data;
  delete[] body_data;
}