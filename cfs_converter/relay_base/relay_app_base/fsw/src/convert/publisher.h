/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef _punlisher_h_
#define _punlisher_h_

#include "define.h"
#include "cfe.h"

/**
 * Create pipe used for message communication.
 */
void publisher_advertise(void);

/**
 * Read the ring buffer, publish unprocessed data.
 * @param ring_buffer   Ring buffer to be read
 * @param next_ring_no  Data id to be read. Read ID next to the latest ID.
 */
void publisher_read_ring_buffer(RosCfeNwBridgeRingBuffer &ring_buffer, unsigned int next_ring_no);

/**
 * Pause with execution cycle.
 */
void publisher_rate_sleep(void);

/**
 * Deserialize data and publish it.
 * @param header    Received data header
 * @param body      Received data body
 */
void publisher_publish(const RosCfeNwBridgeHeader &header, const RosCfeNwBridgeBody &body);

#endif /* _punlisher_h_ */