#ifndef SEQUENCER_H
#define SEQUENCER_H

#include <semaphore.h>
#include <time.h>
#include "dyno.h"

/* ---- Sequencer state (shared with seqgen_main.c) ---- */
extern int                abortTest;
extern int                abortS1;
extern sem_t              semS1;
extern struct timespec    start_time_val;
extern double             start_realtime;
extern unsigned long long sequencePeriods;

/* Thread parameter block */
typedef struct { int threadIdx; } threadParams_t;

/*
 * Initialize sequencer semaphore.
 * Call before creating Service_1 or starting the timer.
 */
void sequencer_init(void);

/*
 * Arm the SIGALRM interval timer at 100 Hz and register Sequencer as handler.
 * Service_1 runs at 50 Hz (every other tick).
 * `periods` — number of 10 ms ticks before automatic shutdown.
 */
void sequencer_start(unsigned long long periods);

/* SIGALRM handler — posts semS1 every 2nd tick, stops timer when done. */
void  Sequencer(int id);

/* RT service thread — 50 Hz state machine. */
void *Service_1(void *threadp);

/* Timing utilities */
double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void   print_scheduler(void);

#endif /* SEQUENCER_H */
