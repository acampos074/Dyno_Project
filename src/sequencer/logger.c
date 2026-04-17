#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <semaphore.h>
#include <pthread.h>

#include "logger.h"

/* ---- Ring buffer ---- */
typedef struct {
    log_entry_t  buf[LOG_RING_SIZE];
    unsigned int head;  /* written by producer (Service_1) only */
    unsigned int tail;  /* written by consumer (logger_thread) only */
} log_ring_t;

static log_ring_t   g_log_ring;
static sem_t        log_sem;
static unsigned int log_dropped = 0;

volatile int log_shutdown = 0;

/* File handles — set by log_init / log_set_dyno_file / log_set_manual_file */
static FILE *g_foc_fp    = NULL;
static FILE *g_dyno_fp   = NULL;
static FILE *g_manual_fp = NULL;
static FILE *g_cal_fp    = NULL;

void log_init(FILE *foc_fp, FILE *dyno_fp)
{
    g_foc_fp  = foc_fp;
    g_dyno_fp = dyno_fp;
    if (sem_init(&log_sem, 0, 0)) {
        printf("log_init: failed to initialize semaphore\n");
        exit(-1);
    }
}

void log_set_dyno_file(FILE *dyno_fp)
{
    /* Called from my_handler (non-RT).  logger_thread reads g_dyno_fp only
     * after sem_wait, which provides the necessary acquire barrier. */
    g_dyno_fp = dyno_fp;
}

void log_set_manual_file(FILE *manual_fp)
{
    g_manual_fp = manual_fp;
}

void log_close_manual_file(void)
{
    if (g_manual_fp) {
        fflush(g_manual_fp);
        fclose(g_manual_fp);
        g_manual_fp = NULL;
    }
}

void log_set_cal_file(FILE *cal_fp)
{
    g_cal_fp = cal_fp;
}

void log_flush_cal(void)
{
    sem_post(&log_sem);
    struct timespec wait = {0, 10000000}; /* 10 ms */
    nanosleep(&wait, NULL);
    if (g_cal_fp) {
        fflush(g_cal_fp);
        fclose(g_cal_fp);
        g_cal_fp = NULL;
    }
}

int log_push(const log_entry_t *e)
{
    unsigned int h = __atomic_load_n(&g_log_ring.head, __ATOMIC_RELAXED);
    unsigned int t = __atomic_load_n(&g_log_ring.tail, __ATOMIC_ACQUIRE);
    if ((h - t) >= LOG_RING_SIZE) {
        log_dropped++;
        return -1;
    }
    g_log_ring.buf[h % LOG_RING_SIZE] = *e;
    __atomic_store_n(&g_log_ring.head, h + 1, __ATOMIC_RELEASE);
    sem_post(&log_sem);
    return 0;
}

void *logger_thread(void *arg)
{
    while (1) {
        sem_wait(&log_sem);

        /* Drain all available entries in one pass */
        unsigned int t = __atomic_load_n(&g_log_ring.tail, __ATOMIC_RELAXED);
        unsigned int h = __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE);

        while (t != h) {
            log_entry_t *e = &g_log_ring.buf[t % LOG_RING_SIZE];
            const char *fmt = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n";
            if (e->log_type == LOG_TYPE_FOC && g_foc_fp)
                fprintf(g_foc_fp, fmt,
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            else if (e->log_type == LOG_TYPE_DYNO && g_dyno_fp)
                fprintf(g_dyno_fp, fmt,
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            else if (e->log_type == LOG_TYPE_MANUAL && g_manual_fp)
                fprintf(g_manual_fp, fmt,
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            else if (e->log_type == LOG_TYPE_CAL && g_cal_fp)
                fprintf(g_cal_fp, fmt,
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            t++;
            h = __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE);
        }
        __atomic_store_n(&g_log_ring.tail, t, __ATOMIC_RELEASE);

        /* Exit only after fully draining the ring */
        if (log_shutdown &&
            __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE) ==
            __atomic_load_n(&g_log_ring.tail, __ATOMIC_ACQUIRE))
            break;
    }

    if (log_dropped)
        printf("logger_thread: dropped %u entries (ring was full)\n", log_dropped);
    return NULL;
}

void log_flush_dyno(void)
{
    /* Poke the logger to drain remaining entries, then close the file.
     * Called from Service_1 when a dyno test finishes — the 10 ms sleep
     * gives logger_thread time to flush before we close the handle. */
    sem_post(&log_sem);
    struct timespec wait = {0, 10000000}; /* 10 ms */
    nanosleep(&wait, NULL);
    if (g_dyno_fp) {
        fflush(g_dyno_fp);
        fclose(g_dyno_fp);
        g_dyno_fp = NULL;
    }
}

void log_shutdown_wait(pthread_t th)
{
    log_shutdown = 1;
    sem_post(&log_sem);
    pthread_join(th, NULL);
}
