#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>

#include <signal.h>

#include <ctype.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <lcm/lcm.h>
#include "../../lib/lcm/FOC_motor_t.h"
#include "../../lib/mcc/pmd.h"
#include "../../lib/mcc/usb-1608FS.h"
#include "../../include/dyno.h"

// ========== SEQUENCER VARIABLES ========
int abortTest=FALSE;
int abortS1=FALSE;
sem_t semS1;
struct timespec start_time_val;
double start_realtime;
unsigned long long sequencePeriods;
static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;
static unsigned long long seqCnt=0;

// ========== SEQUENCER STRUCTURE ========
typedef struct
{
    int threadIdx;
} threadParams_t;

// ========== MCC DAQ VARIABLES ===========
static uint8_t mcc_channel = 0;
static uint8_t mcc_gain = BP_5_00V;
static int mcc_flag,mcc_ret;
static signed short mcc_svalue;
static libusb_device_handle *udev = NULL;
static Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS];
static float NEWTONS_PER_VOLTS = 0.57;
FILE *fp;
FILE *foc_open_loop;
FILE *dyno_test;

// ========== LOG RING BUFFER ==========
// Single-producer (Service_1) / single-consumer (logger_thread) ring buffer.
// Service_1 calls log_push() — a non-blocking struct copy, no file I/O.
// logger_thread drains the ring and writes to disk at its own pace.
// No mutex needed: producer owns `head`, consumer owns `tail`.

#define LOG_RING_SIZE   512     // entries; at 50 Hz this holds ~10 seconds
#define LOG_TYPE_FOC    0       // write to foc_open_loop.csv
#define LOG_TYPE_DYNO   1       // write to dyno_test.csv

typedef struct {
    double  time_s;
    double  vq_cmd;
    float   vq_msr;
    float   iq;
    float   torque_dyno;
    float   velocity;
    float   position;
    float   elec_power;
    float   mech_power;
    float   efficiency;
    int     log_type;
} log_entry_t;

typedef struct {
    log_entry_t buf[LOG_RING_SIZE];
    unsigned int head;   // written by producer (Service_1) only
    unsigned int tail;   // written by consumer (logger_thread) only
} log_ring_t;

static log_ring_t   g_log_ring;
static sem_t        log_sem;
static volatile int log_shutdown = 0;
static unsigned int log_dropped  = 0;   // count of dropped entries (ring full)

// Called from Service_1 — non-blocking, never touches a file descriptor.
static int log_push(const log_entry_t *e)
{
    unsigned int h = __atomic_load_n(&g_log_ring.head, __ATOMIC_RELAXED);
    unsigned int t = __atomic_load_n(&g_log_ring.tail, __ATOMIC_ACQUIRE);
    if ((h - t) >= LOG_RING_SIZE) {
        log_dropped++;
        return -1; // ring full — drop entry rather than block
    }
    g_log_ring.buf[h % LOG_RING_SIZE] = *e;
    __atomic_store_n(&g_log_ring.head, h + 1, __ATOMIC_RELEASE);
    sem_post(&log_sem);
    return 0;
}

// Background logger thread — does all file I/O, runs at normal (non-RT) priority.
void *logger_thread(void *arg)
{
    while (1) {
        sem_wait(&log_sem);

        // Drain all available entries in one go
        unsigned int t = __atomic_load_n(&g_log_ring.tail, __ATOMIC_RELAXED);
        unsigned int h = __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE);
        while (t != h) {
            log_entry_t *e = &g_log_ring.buf[t % LOG_RING_SIZE];
            if (e->log_type == LOG_TYPE_FOC && foc_open_loop)
                fprintf(foc_open_loop,
                        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            else if (e->log_type == LOG_TYPE_DYNO && dyno_test)
                fprintf(dyno_test,
                        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                        e->time_s, e->vq_cmd, e->vq_msr, e->iq,
                        e->torque_dyno, e->velocity, e->position,
                        e->elec_power, e->mech_power, e->efficiency);
            t++;
            h = __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE);
        }
        __atomic_store_n(&g_log_ring.tail, t, __ATOMIC_RELEASE);

        // Exit only after draining everything
        if (log_shutdown &&
            __atomic_load_n(&g_log_ring.head, __ATOMIC_ACQUIRE) ==
            __atomic_load_n(&g_log_ring.tail, __ATOMIC_ACQUIRE))
            break;
    }

    if (log_dropped)
        printf("logger_thread: dropped %u entries (ring was full)\n", log_dropped);
    return NULL;
}

// ========== LCM VARIABLES ======
lcm_t *lcm;
lcm_t *lcm2;

// ========= EVENT HANDLER VARIABLES =========
FOC_motor_t motor_data;
static dyno_t dyno;

// ========= DYNO MUTEX =========
// Protects dyno struct between listener thread (my_handler) and Service_1.
// Rule: lock only long enough to copy in/out — never hold while doing I/O.
static pthread_mutex_t dyno_mutex = PTHREAD_MUTEX_INITIALIZER;

// ========= CANBUS VARIABLES =========
int sock; // Socket descriptor
unsigned char find_can(const int port);
unsigned char data[8];

// ========= LOGGER FUNCTIONS =========
void *logger_thread(void *arg);

// ========= LCM FUNCTIONS =========
void *listener(void* unused);
static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const FOC_motor_t *msg,
                       void *user);

// ========= SEQUENCER FUNCTIONS =========
void Sequencer(int id);
void *Service_1(void *threadp);
double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void print_scheduler(void);

// ========= FLOAT2INT FUNCTIONS =========
int float_to_uint16(float x, float x_min, float x_max);
int float_to_uint12(float x, float x_min, float x_max);
int float_to_uint8(float x, float x_min, float x_max);

float uint16_to_float(int x_int, float x_min, float x_max);
float uint12_to_float(int x_int, float x_min, float x_max);
float uint8_to_float(int x_int, float x_min, float x_max);

// ========= CANBUS FUNCTIONS =========
// Returns 0 on success, -1 on timeout, -2 on error.
// Timeout is CAN_RECV_TIMEOUT_MS (defined in dyno.h).
int  receiveCANFrame(int sock);
void sendCANFrame(int sock, unsigned int canId, unsigned char *data, int dataLength);



int main(void)
{
    // Sequencer local variables
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    int i, rc, scope, flags=0;

    cpu_set_t threadcpu;
    cpu_set_t allcpuset;

    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    pthread_attr_t main_attr;
    pid_t mainpid;

    // Reset dyno variables 
    dyno = (dyno_t) {
        .state = 0,
        .led_flag = 0,
        .pos_cmd = 0.0,
        .vel_cmd = 0.0,
        .vq_cmd = 0.0,
        .iq_cmd = 0.0,
        .torque_cmd = 0.0,
        .kp = 0.0,
        .kd = 0.0,
        .vq_int = 0,
        .iq_int = 0,
        .pos_cmd_int = 0,
        .vel_cmd_int = 0,
        .torque_cmd_int = 0,
        .kp_int = 0,
        .kd_int = 0,
        .sample_waveform = 0.0,
        .torque_dyno = 0.0,
        .elec_power = 0.0,
        .mech_power = 0.0,
        .efficiency = 0.0,
        .total_test_counter = 1,
        .cycle_counter = 1,
        .Fs = 50.0,
        .Vmax = 2,
        .num_cycles = 10,
        .ramp_time = 1.0,
        .meas_time = 6.0,
        .ramp_down_time = 10.0
    };
    // Initialize other dyno variables
    dyno.h = 1/dyno.Fs;
    dyno.ramp_step = (int)(dyno.ramp_time/dyno.h);
    dyno.dv = dyno.Vmax/dyno.ramp_step/dyno.num_cycles;
    dyno.N_ramp_down = (int)(dyno.ramp_down_time/dyno.h);
    dyno.T = (dyno.ramp_time + dyno.meas_time)*dyno.num_cycles + dyno.ramp_down_time;
    dyno.N = (int)(dyno.T/dyno.h);
    printf("h = %f\r\n", dyno.h);
    printf("ramp_step = %d\r\n", dyno.ramp_step);
    printf("dv = %f\r\n", dyno.dv);
    printf("N_ramp_down = %d\r\n", dyno.N_ramp_down);
    printf("T = %f\r\n", dyno.T);
    printf("N = %d\r\n", dyno.N);

    // Open file to write data
    fp = fopen("file.csv", "w");
    if (fp == NULL) {
        printf("Could not open file\n");
        return 1;
    }
    // Write header
    fprintf(fp, "Column1,Column2\n");

    foc_open_loop = fopen("foc_open_loop.csv", "w");
    if (foc_open_loop == NULL) {
        printf("Could not open file foc_open_loop\n");
        return 1;
    }
    /*
    dyno_test = fopen("dyno_test.csv", "w");
    if (dyno_test == NULL) {
        printf("Could not open file dyno_test\n");
        return 1;
    }*/
    // Write header
    fprintf(foc_open_loop,"Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
    //fprintf(dyno_test,"Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
    
    //fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),motor_data.vq,(motor_data.iq),(torque_dyno),(motor_data.velocity),motor_data.position,(elec_power),(mech_power),(efficiency));

    // ============= MCC Code =============
    // UNCOMMENT TO COLLECT DATA FROM MCC DAQ
  
    mcc_ret = libusb_init(NULL);
    if (mcc_ret < 0) 
    {
      perror("libusb_init: Failed to initialize libusb");
      exit(1);
    }

    if ((udev = usb_device_find_USB_MCC(USB1608FS_PID, NULL))) 
    {
      printf("USB-1608FS Device is found!\n");
    } else {
      printf("No device found.\n");
      exit(0);
    }

    printf("Building calibration table.  This may take a while ...\n");
    usbBuildCalTable_USB1608FS(udev, table_AIN);

  //usbDConfigPort_USB1608FS(udev, DIO_DIR_OUT);
  //mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
  // EXAMPLE OF HOW TO READ ANALOG INPUT AND STORE VALUE INTO LCM VARIABLE:
  //usb_pkg_analog_t my_data;
  //my_data.signal = volts_USB1608FS(gain, svalue);

  // ========== LCM CODE ==========
    // Create an LCM instance
    lcm = lcm_create(NULL);
    if (!lcm)
        return 1;
    
    lcm2 = lcm_create(NULL);
    if (!lcm2)
        return 1;
        
    //my_data.x = 15;
    //my_data.y = 11;
    
    FOC_motor_t_subscribe(lcm2, "MOTOR", &my_handler, NULL);
    pthread_t th1;
    pthread_create(&th1, NULL, listener, NULL);

    // Start logger thread at normal (non-RT) priority — all file I/O happens here
    pthread_t th_logger;
    pthread_create(&th_logger, NULL, logger_thread, NULL);

  // ========= CANBUS CODE =========
    int ret;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    memset(&frame,0,sizeof(struct can_frame));
    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ifconfig can0 up");
    printf("This is a socket can transmit & receive demo program ,can0 with 1Mbps baud rate\r\n");
    
    // Create a socket
    sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock == -1) {
        perror("Failed to create socket");
        return 1;
    }

    // Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(sock, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl interface index failed!");
        return 1;
    }
    
    // Bind the socket to the can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        perror("Failed to bind socket");
        close(sock);
        return 1;
    }

    struct can_filter rfilter[2];
    rfilter[0].can_id = 0x123;//Standard frame id !
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x12345678;//extend frame id!
    rfilter[1].can_mask = CAN_EFF_MASK;

    // Prepare CAN frame
    /*assembly  message data! */
    frame.can_id = 0x123;
    frame.can_dlc = 8; // Using 2 bytes to encode a single torque value,1 for led flag,1 for general command, 1 for speed cmd. Max of 8
    frame.data[0] = 1;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    if(!(frame.can_id&CAN_EFF_FLAG))
        printf("Transmit standard frame!\n");
    else
    {
        printf("Transmit extended frame!\n");
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);
    }

    // prints data frame
    for(i = 0; i < frame.can_dlc; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);

    // Initial handshake — send and receive one frame to verify CAN is up
    sendCANFrame(sock, CAN_ID, data, 6);
    receiveCANFrame(sock); // timeout here is non-fatal at startup

    // ========== SEQUENCER CODE ==========

    printf("Starting High Rate Sequencer Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val); start_realtime=realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res); current_realtime_res=realtime(&current_time_res);
    printf("START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);

   printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    if (sem_init(&semS1,   0, 0)) { printf("Failed to initialize S1 semaphore\n");     exit(-1); }
    if (sem_init(&log_sem, 0, 0)) { printf("Failed to initialize log semaphore\n");    exit(-1); }

    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();

    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);


    for(i=0; i < NUM_THREADS; i++)
    {

      // run even indexed threads on core 2
      if(i % 2 == 0)
      {
          CPU_ZERO(&threadcpu);
          cpuidx=(2);
          CPU_SET(cpuidx, &threadcpu);
      }

      // run odd indexed threads on core 3
      else
      {
          CPU_ZERO(&threadcpu);
          cpuidx=(3);
          CPU_SET(cpuidx, &threadcpu);
      }

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1 = RT_MAX-1	@ 50 Hz
    //
    rt_param[0].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[0]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");

    sleep(1);
 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    printf("Start sequencer\n");
    sequencePeriods=2000;

    // Sequencer = RT_MAX	@ 100 Hz
    //
    /* set up to signal SIGALRM if timer expires */
    timer_create(CLOCK_MONOTONIC, NULL, &timer_1);

    signal(SIGALRM, (void(*)()) Sequencer);


    /* arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 10000000; // 10ms interval or 100hz sequencer
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 10000000;
    //itime.it_interval.tv_sec = 1;
    //itime.it_interval.tv_nsec = 0;
    //itime.it_value.tv_sec = 1;
    //itime.it_value.tv_nsec = 0;

    timer_settime(timer_1, flags, &itime, &last_itime);


    for(i=0;i<NUM_THREADS;i++)
    {
        if(rc=pthread_join(threads[i], NULL) < 0)
		perror("main pthread_join");
	else
		printf("joined thread %d\n", i);
    }

    // Signal logger thread to flush remaining entries and exit, then wait for it
    log_shutdown = 1;
    sem_post(&log_sem);
    pthread_join(th_logger, NULL);

    fclose(fp); fclose(foc_open_loop);
    close(sock);
    printf("exiting from main program\n");
    pthread_join(th1, NULL);
    // Cleanup and exit
    lcm_destroy(lcm);
    lcm_destroy(lcm2);

    printf("\nTEST COMPLETE\n");

   return 0;
}


void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags=0;

    // received interval timer signal
           
    seqCnt++;
    // Reset seqCnt if it reaches 1000
    if (seqCnt >= 1000) {
        seqCnt = 1;
    }



    // Release each service at a sub-rate of the generic sequencer rate

    // Servcie_1 = RT_MAX-1	@ 50 Hz
    if((seqCnt % 2) == 0) sem_post(&semS1);

    if(abortTest || (seqCnt >= sequencePeriods))
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
	printf("Disabling sequencer interval timer with abort=%d and %llu of %llu\n", abortTest, seqCnt, sequencePeriods);

	// shutdown all services
        sem_post(&semS1); 

        abortS1=TRUE; 
    }

}

void *Service_1(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    printf("S1 thread started @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS1) // check for synchronous abort request
    {
        // wait for service request from the sequencer
        sem_wait(&semS1);

        S1Cnt++;

        // --- Snapshot: lock only long enough to copy command fields ---
        // Service_1 never holds dyno_mutex during I/O (CAN, MCC, file writes).
        dyno_t cmd;
        pthread_mutex_lock(&dyno_mutex);
        cmd = dyno; // full struct copy
        pthread_mutex_unlock(&dyno_mutex);

        // DO WORK (50Hz state machine) using the local snapshot (cmd.*).
        // All reads use cmd.* — no lock needed.
        // Write-backs to dyno.* happen under lock at the end of each case.
        switch(cmd.state){
            case CMD_CALIBRATE:  // 1
                memset(data, 0, sizeof(data));
                data[0] = 0;
                data[1] = CMD_CALIBRATE;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_NONE;
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            case CMD_OFF:       // 2
                memset(data, 0, sizeof(data));
                data[0] = 0;
                data[1] = CMD_OFF;
                sendCANFrame(sock, CAN_ID, data, 8);
                if (receiveCANFrame(sock) == 0) {
                    motor_data.cmd_id = CMD_NONE;
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                }
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            case CMD_VOLTAGE_FOC: // 3
            {
                int vq_int = float_to_uint16(cmd.vq_cmd, -V_MAX, V_MAX);
                memset(data, 0, sizeof(data));
                data[1] = CMD_VOLTAGE_FOC;
                data[2] = vq_int >> 8;
                data[3] = vq_int & 0xFF;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_VOLTAGE_FOC;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
                float torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue) * NEWTONS_PER_VOLTS;
                float elec_power  = motor_data.iq * cmd.vq_cmd;
                float mech_power  = torque_dyno * motor_data.velocity;
                float efficiency  = mech_power / elec_power;
                clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime = realtime(&current_time_val);
                log_push(&(log_entry_t){
                    .log_type   = LOG_TYPE_FOC,
                    .time_s     = current_realtime - start_realtime,
                    .vq_cmd     = cmd.vq_cmd,
                    .vq_msr     = motor_data.vq,
                    .iq         = motor_data.iq,
                    .torque_dyno= torque_dyno,
                    .velocity   = motor_data.velocity,
                    .position   = motor_data.position,
                    .elec_power = elec_power,
                    .mech_power = mech_power,
                    .efficiency = efficiency
                });
                // Write computed metrics back under lock
                pthread_mutex_lock(&dyno_mutex);
                dyno.torque_dyno = torque_dyno;
                dyno.elec_power  = elec_power;
                dyno.mech_power  = mech_power;
                dyno.efficiency  = efficiency;
                pthread_mutex_unlock(&dyno_mutex);
                break;
            }

            case CMD_POSITION:  // 4
            {
                int pos_int = float_to_uint16(cmd.pos_cmd, -ONE_REV*GR, ONE_REV*GR);
                int kp_int  = float_to_uint8(cmd.kp, 0.0, KP_MAX/GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0, 0.1/GAIN);
                memset(data, 0, sizeof(data));
                data[0] = CMD_POSITION;
                data[1] = CMD_POSITION;
                data[2] = pos_int >> 8;
                data[3] = pos_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_POSITION;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            }

            case CMD_VELOCITY:  // 5
            {
                int vel_int = float_to_uint16(cmd.vel_cmd, -SPEED_MAX, SPEED_MAX);
                int kp_int  = float_to_uint8(cmd.kp, 0.0, KP_MAX/GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0, 0.1/GAIN);
                memset(data, 0, sizeof(data));
                data[0] = 0;
                data[1] = CMD_VELOCITY;
                data[2] = vel_int >> 8;
                data[3] = vel_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_VELOCITY;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            }

            case CMD_DYNO_TEST:  // 6
            {
                // Update voltage waveform using snapshot values
                double vq_cmd = cmd.vq_cmd;
                int    cycle  = cmd.cycle_counter;
                int    total  = cmd.total_test_counter;

                if (total <= cmd.N - cmd.N_ramp_down) {
                    if (cycle <= (int)(cmd.ramp_time / cmd.h))
                        vq_cmd = vq_cmd - cmd.dv;
                    cycle++;
                    if (cycle > (int)(cmd.ramp_time / cmd.h) + (int)(cmd.meas_time / cmd.h))
                        cycle = 1;
                } else {
                    vq_cmd = vq_cmd + cmd.dv;
                }

                int test_done = (total >= cmd.N);
                if (test_done) {
                    vq_cmd = 0.0;
                    total  = 1;
                    cycle  = 0;
                }

                int vq_int = float_to_uint16(vq_cmd, -V_MAX, V_MAX);
                memset(data, 0, sizeof(data));
                data[1] = test_done ? CMD_OFF : CMD_DYNO_TEST;
                data[2] = vq_int >> 8;
                data[3] = vq_int & 0xFF;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = test_done ? CMD_OFF : CMD_DYNO_TEST;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);

                mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
                float torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue) * NEWTONS_PER_VOLTS;
                float elec_power  = motor_data.iq * vq_cmd;
                float mech_power  = torque_dyno * motor_data.velocity;
                float efficiency  = mech_power / elec_power;
                clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime = realtime(&current_time_val);
                log_push(&(log_entry_t){
                    .log_type   = LOG_TYPE_DYNO,
                    .time_s     = current_realtime - start_realtime,
                    .vq_cmd     = vq_cmd,
                    .vq_msr     = motor_data.vq,
                    .iq         = motor_data.iq,
                    .torque_dyno= torque_dyno,
                    .velocity   = motor_data.velocity,
                    .position   = motor_data.position,
                    .elec_power = elec_power,
                    .mech_power = mech_power,
                    .efficiency = efficiency
                });

                if (test_done) {
                    // Flush the ring to disk before closing the file.
                    // Signal logger and give it a moment to drain.
                    sem_post(&log_sem);
                    struct timespec flush_wait = {0, 10000000}; // 10 ms
                    nanosleep(&flush_wait, NULL);
                    fclose(dyno_test);
                    dyno_test = NULL;
                }

                // Write back mutable fields under lock
                pthread_mutex_lock(&dyno_mutex);
                dyno.vq_cmd           = vq_cmd;
                dyno.cycle_counter    = cycle;
                dyno.total_test_counter = total;
                dyno.torque_dyno      = torque_dyno;
                dyno.elec_power       = elec_power;
                dyno.mech_power       = mech_power;
                dyno.efficiency       = efficiency;
                if (test_done) dyno.state = CMD_OFF;
                pthread_mutex_unlock(&dyno_mutex);
                break;
            }

            case CMD_TOGGLE_LED: // 7
                memset(data, 0, sizeof(data));
                data[0] = cmd.led_flag;
                data[1] = CMD_TOGGLE_LED;
                sendCANFrame(sock, CAN_ID, data, 8);
                if (receiveCANFrame(sock) == 0) {
                    motor_data.cmd_id = CMD_NONE;
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                }
                pthread_mutex_lock(&dyno_mutex);
                dyno.state = CMD_NONE;
                pthread_mutex_unlock(&dyno_mutex);
                break;

            case CMD_CURRENT:   // 8
            {
                int iq_int = float_to_uint16(cmd.iq_cmd, -I_MAX, I_MAX);
                memset(data, 0, sizeof(data));
                data[1] = CMD_CURRENT;
                data[2] = iq_int >> 8;
                data[3] = iq_int & 0xFF;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_CURRENT;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            }

            case CMD_TORQUE:    // 9
            {
                int tau_int = float_to_uint16(cmd.torque_cmd, -TAU_MAX, TAU_MAX);
                int kp_int  = float_to_uint8(cmd.kp, 0.0, KP_MAX/GAIN);
                int kd_int  = float_to_uint8(cmd.kd, 0.0, 0.1/GAIN);
                memset(data, 0, sizeof(data));
                data[0] = 0;
                data[1] = CMD_TORQUE;
                data[2] = tau_int >> 8;
                data[3] = tau_int & 0xFF;
                data[4] = kp_int;
                data[5] = kd_int;
                sendCANFrame(sock, CAN_ID, data, 8);
                motor_data.cmd_id = CMD_TORQUE;
                if (receiveCANFrame(sock) == 0)
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            }

            default:
                break;
        }
        

    }

    // Resource shutdown here
    //
    pthread_exit((void *)0);
}

double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(MY_CLOCK_TYPE, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}


double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}


void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
       case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
       case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n"); exit(-1);
         break;
       case SCHED_RR:
           printf("Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
       //case SCHED_DEADLINE:
       //    printf("Pthread Policy is SCHED_DEADLINE\n"); exit(-1);
       //    break;
       default:
           printf("Pthread Policy is UNKNOWN\n"); exit(-1);
   }
}

void *listener(void* unused)
{
   while (1)
   {
      lcm_handle(lcm2);
   }
    //printf("Thread %s done!\n", name);
    return NULL;
}

static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const FOC_motor_t *msg,
                       void *user)
{
    // Open dyno_test file outside the lock — file I/O must not hold dyno_mutex.
    FILE *new_dyno_test = NULL;
    if ((MotorCmd)msg->cmd_id == CMD_DYNO_TEST)
    {
        new_dyno_test = fopen("dyno_test.csv", "w");
        if (new_dyno_test == NULL) {
            printf("Could not open file dyno_test\n");
            return;
        }
        fprintf(new_dyno_test, "Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),"
                               "Torque (Nm),Speed (rad/s),Pos (rad),"
                               "Elec Power (W),Mech Power (W),Efficiency\n");
    }

    // Lock only while writing to dyno — no I/O inside the lock.
    pthread_mutex_lock(&dyno_mutex);
    switch((MotorCmd)msg->cmd_id)
    {
        case CMD_CALIBRATE:
            dyno.state = CMD_CALIBRATE;
            break;
        case CMD_OFF:
            dyno.state = CMD_OFF;
            break;
        case CMD_VOLTAGE_FOC:
            dyno.state = CMD_VOLTAGE_FOC;
            dyno.vq_cmd = msg->vq_cmd;
            break;
        case CMD_POSITION:
            dyno.state = CMD_POSITION;
            dyno.pos_cmd = msg->position_cmd;
            dyno.kp = msg->kp;
            dyno.kd = msg->kd;
            break;
        case CMD_VELOCITY:
            dyno.state = CMD_VELOCITY;
            dyno.vel_cmd = msg->velocity_cmd;
            dyno.kp = msg->kp;
            dyno.kd = msg->kd;
            break;
        case CMD_DYNO_TEST:
            dyno.state = CMD_DYNO_TEST;
            dyno.vq_cmd = -dyno.dv; // start from zero - dv
            dyno_test = new_dyno_test;
            break;
        case CMD_TOGGLE_LED:
            dyno.led_flag = msg->led;
            dyno.state = CMD_TOGGLE_LED;
            break;
        case CMD_CURRENT:
            dyno.iq_cmd = msg->iq_cmd;
            dyno.state = CMD_CURRENT;
            break;
        case CMD_TORQUE:
            dyno.torque_cmd = msg->torque_cmd;
            dyno.state = CMD_TORQUE;
            dyno.kp = msg->kp;
            dyno.kd = msg->kd;
            break;
        default:
            break;
    }
    pthread_mutex_unlock(&dyno_mutex);
}

int float_to_uint8(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<8)-1))/span);
}

int float_to_uint16(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<16)-1))/span);
    //return ((int) (x-offset)*((float)((1<<16)-1))/span);
}

int float_to_uint12(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<12)-1))/span);
    //return ((int) (x-offset)*((float)((1<<16)-1))/span);
}

float uint16_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<16)-1)) + offset;
}

float uint12_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<12)-1)) + offset;
}

float uint8_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<8)-1)) + offset;
}

void sendCANFrame(int sock, unsigned int canId, unsigned char *data, int dataLength) {
    struct can_frame frame;
    frame.can_id = canId;
    frame.can_dlc = dataLength;
    memcpy(frame.data, data, dataLength);

    if (write(sock, &frame, sizeof(struct can_frame)) == -1) {
        perror("Failed to write CAN frame");
        close(sock);
        exit(1);
    }
}

int receiveCANFrame(int sock) {
    // Wait up to CAN_RECV_TIMEOUT_MS for a frame before giving up.
    // This prevents Service_1 from blocking indefinitely if the motor
    // controller does not reply (e.g. disconnected, bus off).
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = CAN_RECV_TIMEOUT_MS * 1000;

    int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (ret == 0) {
        // Timeout — no frame arrived within the deadline.
        printf("receiveCANFrame: timeout after %d ms\n", CAN_RECV_TIMEOUT_MS);
        return -1;
    }
    if (ret < 0) {
        perror("receiveCANFrame: select failed");
        return -2;
    }

    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("receiveCANFrame: read failed");
        return -2;
    }

    motor_data.position = uint16_to_float((frame.data[0]<<8 | frame.data[1]), -ONE_REV*GR, ONE_REV*GR);
    motor_data.velocity = uint16_to_float((frame.data[2]<<8 | frame.data[3]), -SPEED_MAX,  SPEED_MAX);
    motor_data.vq       = uint16_to_float((frame.data[4]<<8 | frame.data[5]), -V_MAX,      V_MAX);
    motor_data.iq       = uint16_to_float((frame.data[6]<<8 | frame.data[7]), -I_MAX,      I_MAX);
    motor_data.torque   = motor_data.iq * KT; // gear ratio cancels: KT*GR/GR
    return 0;
}

