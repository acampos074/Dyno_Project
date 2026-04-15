#ifndef LCM_INTERFACE_H
#define LCM_INTERFACE_H

#include <pthread.h>
#include <lcm/lcm.h>
#include "FOC_motor_t.h"
#include "dyno.h"

/* LCM channel names shared between GUI and sequencer */
#define LCM_CHAN_GUI    "GUI"    /* sequencer → GUI: motor state    */
#define LCM_CHAN_MOTOR  "MOTOR"  /* GUI → sequencer: commands       */

/*
 * Shared state — written by my_handler (listener thread),
 * read by Service_1 (RT thread), protected by dyno_mutex.
 */
extern lcm_t          *lcm;         /* publish motor state on LCM_CHAN_GUI   */
extern lcm_t          *lcm2;        /* subscribe commands on LCM_CHAN_MOTOR  */
extern FOC_motor_t     motor_data;  /* last received CAN frame, decoded      */
extern dyno_t          dyno;        /* sequencer command state               */
extern pthread_mutex_t dyno_mutex;  /* protects dyno and motor_data          */

/*
 * Initialize dyno struct with default operating parameters.
 * Must be called before lcm_interface_init().
 */
void dyno_init(void);

/*
 * Set the directory where dyno_test.csv and manual log files are created.
 * Call before starting the listener thread.
 */
void lcm_interface_set_data_dir(const char *dir);

/*
 * Create both LCM instances and subscribe my_handler to LCM_CHAN_MOTOR.
 * Returns 0 on success, -1 on failure.
 */
int  lcm_interface_init(void);

/* LCM listener thread — calls lcm_handle(lcm2) in a loop. */
void *listener(void *unused);

#endif /* LCM_INTERFACE_H */
