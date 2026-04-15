#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "FOC_motor_t.h"

/*
 * Open, configure, and bind a SocketCAN socket for can0 at 1 Mbps.
 * Owns the socket internally; other functions use it without a fd argument.
 * Returns 0 on success, -1 on error.
 */
int  can_open(void);

/* Release the socket. */
void can_close(void);

/*
 * Send a raw CAN frame.
 * Calls exit(1) on write failure (unrecoverable — bus is unusable).
 */
void sendCANFrame(unsigned int canId, unsigned char *data, int dataLength);

/*
 * Receive one CAN frame with a timeout of CAN_RECV_TIMEOUT_MS (from dyno.h).
 * If `out` is non-NULL, motor state decoded from frame bytes is written there.
 *
 * Returns:
 *   0  — frame received and decoded into *out
 *  -1  — timeout (no frame within deadline)
 *  -2  — select/read error
 */
int  receiveCANFrame(FOC_motor_t *out);

#endif /* CAN_DRIVER_H */
