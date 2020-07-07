/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _udp_h_
#define _udp_h_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

/**
 * Init UDP server.
 * @param ip_addr IP address
 * @param port    Port number
 * @return File descriptor of generated socket
 */
int8_t udp_server_init(const char* ip_addr, unsigned short port);

/**
 * Init UDP client.
 * @param ip_addr IP address
 * @param port    Port number
 * @return File descriptor of generated socket
 */
int8_t udp_client_init(const char* ip_addr, unsigned short port);

/**
 * Receive data with UDP/IP.
 * @param dst_sock  File descriptor of socket
 * @param buf_size  Received data size
 * @param read_data Received data
 * @return Result
 */
int8_t udp_read(int8_t dst_sock, unsigned int buf_size, unsigned char *read_data);

/**
 * Send data with UDP/IP.
 * @param dst_sock      File descriptor of socket
 * @param write_data    Sent data
 * @param data_size     Sent data size
 * @param ipaddr        IP address
 * @param port          Port number
 * @param ret           Result (0: success, -1: error)
 * @return Sent data size. If error occurs, return 0.
 */
uint32_t udp_write(int8_t dst_sock, uint8_t* write_data, uint32_t data_size, const char* ipaddr, uint16_t port, char *ret);

/**
 * Close socket.
 * @param dst_sock File descriptor of socket
 * @return Result
 */
int8_t udp_close(int8_t dst_sock);

#endif // _udp_h_
