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

/**
 * Message header structure
 */
typedef struct {
  unsigned int body_len;
  double unix_time;
  unsigned char sender;
  unsigned char send_no;
  unsigned short msg_id;
} RosCfeNwBridgeHeader;

/**
 * Message body structure
 */
typedef struct { unsigned char data[MAX_BODY_LEN]; } RosCfeNwBridgeBody;

/**
 * Structure of an element of ring buffer
 */
typedef struct {
  RosCfeNwBridgeHeader header;
  RosCfeNwBridgeBody body;
  unsigned char is_treated;
} RosCfeNwBridgeBuffer;

/**
 * Structure of ring buffer
 */
typedef struct {
  // RosCfeNwBridgeBuffer buf[RING_BUF_LEN];
  RosCfeNwBridgeBuffer *buf;
} RosCfeNwBridgeRingBuffer;

/**
 * Deserialize message header.
 * @param buffer Data to be deserialized
 * @return Deserialized data
 */
RosCfeNwBridgeHeader deserialize_ros_cfe_header(unsigned char *buffer);

/**
 * Serialize message header.
 * @param header Data to be serialized
 * @param buffer Serialized data
 * @return Data length
 */
int serialize_ros_cfe_header(const RosCfeNwBridgeHeader &header,
                             unsigned char *buffer);

#endif /* _define_h_ */
