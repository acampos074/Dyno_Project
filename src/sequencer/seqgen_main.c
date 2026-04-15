#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>

#include "sequencer.h"
#include "can_driver.h"
#include "mcc_driver.h"
#include "logger.h"
#include "lcm_interface.h"
#include "dyno.h"

int main(void)
{
    int i, rc, scope;
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    /* ---- Create timestamped run directory ---- */
    char run_dir[256];
    {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        char ts[32];
        strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", tm_info);

        /* Ensure top-level data/ directory exists */
        if (mkdir("data", 0755) < 0 && errno != EEXIST) {
            perror("mkdir data");
            return 1;
        }

        snprintf(run_dir, sizeof(run_dir), "data/run_%s", ts);
        if (mkdir(run_dir, 0755) < 0) {
            perror("mkdir run_dir");
            return 1;
        }
        printf("Run directory: %s\n", run_dir);
    }

    /* ---- Init shared dyno state ---- */
    dyno_init();
    printf("h = %f\n",            dyno.h);
    printf("ramp_step = %d\n",    dyno.ramp_step);
    printf("dv = %f\n",           dyno.dv);
    printf("N_ramp_down = %d\n",  dyno.N_ramp_down);
    printf("T = %f\n",            dyno.T);
    printf("N = %d\n",            dyno.N);

    /* ---- Open CSV log files inside run directory ---- */
    char path_buf[320];

    snprintf(path_buf, sizeof(path_buf), "%s/file.csv", run_dir);
    FILE *fp = fopen(path_buf, "w");
    if (!fp) { printf("Cannot open %s\n", path_buf); return 1; }
    fprintf(fp, "Column1,Column2\n");

    snprintf(path_buf, sizeof(path_buf), "%s/foc_open_loop.csv", run_dir);
    FILE *foc_open_loop = fopen(path_buf, "w");
    if (!foc_open_loop) { printf("Cannot open %s\n", path_buf); return 1; }
    fprintf(foc_open_loop,
            "Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),"
            "Torque (Nm),Speed (rad/s),Pos (rad),"
            "Elec Power (W),Mech Power (W),Efficiency\n");

    /* ---- Module init ---- */
    if (mcc_init() < 0) return 1;
    lcm_interface_set_data_dir(run_dir);
    if (lcm_interface_init() < 0) return 1;
    if (can_open() < 0) return 1;

    log_init(foc_open_loop, NULL);
    sequencer_init();

    /* ---- CAN handshake ---- */
    unsigned char init_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    sendCANFrame(CAN_ID, init_data, 6);
    receiveCANFrame(NULL); /* timeout here is non-fatal at startup */

    /* ---- RT scheduling setup ---- */
    printf("Starting High Rate Sequencer\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val);
    start_realtime = realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res);
    current_realtime_res = realtime(&current_time_res);
    printf("START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n",
           (current_realtime - start_realtime), current_realtime_res);

    printf("System has %d processors configured and %d available.\n",
           get_nprocs_conf(), get_nprocs());

    cpu_set_t allcpuset;
    CPU_ZERO(&allcpuset);
    for (i = 0; i < NUM_CPU_CORES; i++)
        CPU_SET(i, &allcpuset);
    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

    pthread_t          threads[NUM_THREADS];
    threadParams_t     threadParams[NUM_THREADS];
    pthread_attr_t     rt_sched_attr[NUM_THREADS];
    struct sched_param rt_param[NUM_THREADS];
    int rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    int rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    struct sched_param main_param;
    pthread_attr_t main_attr;
    pid_t mainpid = getpid();

    rc = sched_getparam(mainpid, &main_param);
    main_param.sched_priority = rt_max_prio;
    rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if (rc < 0) perror("main_param");
    print_scheduler();

    pthread_attr_getscope(&main_attr, &scope);
    if (scope == PTHREAD_SCOPE_SYSTEM)
        printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
        printf("PTHREAD SCOPE PROCESS\n");
    else
        printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    cpu_set_t threadcpu;
    int cpuidx;

    for (i = 0; i < NUM_THREADS; i++) {
        CPU_ZERO(&threadcpu);
        cpuidx = (i % 2 == 0) ? 2 : 3;
        CPU_SET(cpuidx, &threadcpu);

        rc = pthread_attr_init(&rt_sched_attr[i]);
        rc = pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rc = pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        rt_param[i].sched_priority = rt_max_prio - i;
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
        threadParams[i].threadIdx = i;
    }
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    /* ---- Create logger and LCM listener threads (non-RT) ---- */
    pthread_t th_listener, th_logger;
    pthread_create(&th_listener, NULL, listener,      NULL);
    pthread_create(&th_logger,   NULL, logger_thread, NULL);

    /* ---- Create Service_1 at RT_MAX-1, 50 Hz ---- */
    rt_param[0].sched_priority = rt_max_prio - 1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc = pthread_create(&threads[0], &rt_sched_attr[0],
                        Service_1, (void *)&(threadParams[0]));
    if (rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");

    sleep(1);

    /* ---- Start sequencer timer at 100 Hz, 2000 ticks (20 s) ---- */
    printf("Start sequencer\n");
    sequencer_start(2000);

    /* ---- Wait for Service_1 to finish ---- */
    for (i = 0; i < NUM_THREADS; i++) {
        if ((rc = pthread_join(threads[i], NULL)) < 0)
            perror("main pthread_join");
        else
            printf("joined thread %d\n", i);
    }

    /* ---- Flush logger and clean up ---- */
    log_close_manual_file();   /* ensure manual log is saved if still open */
    log_shutdown_wait(th_logger);

    fclose(fp);
    fclose(foc_open_loop);
    can_close();
    mcc_close();
    pthread_join(th_listener, NULL);
    lcm_destroy(lcm);
    lcm_destroy(lcm2);

    printf("\nTEST COMPLETE\n");
    return 0;
}
