/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _define_h_
#define _define_h_

#define MAX_BODY_LEN 1048576
#define TREATED 1
#define NOT_TREAT 0
#define SIZEOF_HEADER 16

#define RING_BUF_LEN $$_RING_BUF_LEN_$$

/**
 * Communication header structure
 */
typedef struct {
  unsigned int body_len;
  double unix_time;
  unsigned char sender;
  unsigned char send_no;
  unsigned short msg_id;
} RosCfeNwBridgeHeader;

/**
 * Communication body structure
 */
typedef struct {
  unsigned char data[MAX_BODY_LEN];
} RosCfeNwBridgeBody;

/**
 * One-element structure of the ring buffer
 */
typedef struct {
  RosCfeNwBridgeHeader header;
  RosCfeNwBridgeBody body;
  unsigned char is_treated;
} RosCfeNwBridgeBuffer;

/**
 * Ring buffer structure
 */
typedef struct {
  RosCfeNwBridgeBuffer buf[RING_BUF_LEN];
} RosCfeNwBridgeRingBuffer;

/**
 * Deserialize communication header
 * @param buffer Data to deserialize
 * @return Deserialize result
 */
RosCfeNwBridgeHeader deserialize_ros_cfe_header(unsigned char *buffer);

/**
 * Serialize communication header
 * @param header Data to serialize
 * @param buffer Serialize result
 * @return Data size
 */
int serialize_ros_cfe_header(const RosCfeNwBridgeHeader &header, unsigned char *buffer);

#endif /* _define_h_ */
