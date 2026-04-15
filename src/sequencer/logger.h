#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <pthread.h>

#define LOG_RING_SIZE  512   /* at 50 Hz, holds ~10 s of data   */
#define LOG_TYPE_FOC   0     /* entry goes to foc_open_loop.csv */
#define LOG_TYPE_DYNO  1     /* entry goes to dyno_test.csv     */

typedef struct {
    double time_s;
    double vq_cmd;
    float  vq_msr;
    float  iq;
    float  torque_dyno;
    float  velocity;
    float  position;
    float  elec_power;
    float  mech_power;
    float  efficiency;
    int    log_type;
} log_entry_t;

/*
 * Initialize the logger ring buffer and semaphore.
 * `foc_fp` must be a valid open FILE*.
 * `dyno_fp` may be NULL at startup (set later via log_set_dyno_file).
 * Call before creating the logger thread.
 */
void log_init(FILE *foc_fp, FILE *dyno_fp);

/*
 * Update the dyno_test file handle (call when CMD_DYNO_TEST starts).
 * Not called from the RT thread.
 */
void log_set_dyno_file(FILE *dyno_fp);

/*
 * Push one entry to the ring buffer — non-blocking, never touches a file.
 * Safe to call from the RT Service_1 thread.
 * Returns 0 on success, -1 if the ring was full (entry dropped).
 */
int log_push(const log_entry_t *e);

/* Background logger thread — drains the ring to disk, exits on log_shutdown. */
void *logger_thread(void *arg);

/*
 * Called by Service_1 (CMD_DYNO_TEST, test_done path) to flush the ring,
 * wait a brief period, then close and NULL the dyno_test file handle.
 */
void log_flush_dyno(void);

/* Signal logger_thread to flush remaining entries and exit, then join it. */
void log_shutdown_wait(pthread_t th);

/* Set to 1 externally to tell logger_thread to exit after draining. */
extern volatile int log_shutdown;

#endif /* LOGGER_H */
