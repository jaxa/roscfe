/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _communication_h_
#define _communication_h_

#define IP_ADDR $$_COM_IP_ADDR_$$
#define PORT_NO $$_PORT_$$
#define PORT_NO_SEND $$_PORT_SEND_$$
#define PORT_NO_RECV $$_PORT_RECV_$$

// definition
#define PROTOCOL_TCP 0
#define PROTOCOL_UDP 1
#define COM_SUCCESS 0
#define COM_ERROR -1

/**
 * Init communication.
 * @return Result(0: success, -1: failure)
 */
int communication_init(void);

/**
 * Write data.
 * @param size      Written data size
 * @param buffer    Written data
 * @param ret       Result (0: success, -1: error)
 * @return Written data size
 */
unsigned int communication_write(unsigned int size, unsigned char *buffer, char *ret);

/**
 * Read data from socket
 * @param size      Read data size
 * @param buffer    Read data
 * @return Result (0: success, -1: failure)
 */
int communication_read(unsigned int size, unsigned char *buffer);

/**
 * Close communication.
 * @return Result (0: success, -1: failure)
 */
int communication_close(void);

/**
 * Advertise topic.
 */
void communication_advertise(void);

/**
 * Read data from ring buffer, publish data.
 */
void communication_publish_recv_data(void);

/**
 * Receive message from CFS apps, store in the ring buffer.
 * This function runs in the background.
 * @param argv Arguments passed to this function
 */
void* communication_read_callback(void* argv);

#endif /* _communication_h_ */
