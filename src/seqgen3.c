#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
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
//#include "mypackage_Position.h"
#include "FOC_motor_t.h"

#include "pmd.h"
#include "usb-1608FS.h"

#include "dyno.h"

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
static float NEWTONS_PER_VOLTS = 0.57;//2.85/5.0;
static float GAIN = 1.0; // changed from 25 to 1 on 07/13/25
static float kp_max = 0.2f;
FILE *fp;
FILE *foc_open_loop;
FILE *dyno_test;

// ========== LCM VARIABLES ======
lcm_t *lcm;
lcm_t *lcm2;

// ========= EVENT HANDLER VARIABLES =========
//mypackage_Position my_data;
FOC_motor_t motor_data;
static dyno_t dyno;

// ========= CANBUS VARIABLES =========
int sock; // Socket descriptor
unsigned char find_can(const int port);
unsigned char data[8];

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
void receiveCANFrame(int sock); 
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
    
    //fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),motor_data.vq,(motor_data.iq),(torque_dyno),(motor_data.velocity),motor_data.position,(elec_power),(mech_power),(efficiency));

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
    // Start a second thread to listen for LCM messages
    pthread_create(&th1, NULL, listener, NULL); 

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

    // Send a message from sock1 to sock2
    sendCANFrame(sock, 0x123, data, 6);

    // Receive the message on sock2
    receiveCANFrame(sock);

    // ========== SEQUENCER CODE ==========

    printf("Starting High Rate Sequencer Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val); start_realtime=realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res); current_realtime_res=realtime(&current_time_res);
    printf("START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);
    syslog(LOG_CRIT, "START High Rate Sequencer @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);

   printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

   CPU_ZERO(&allcpuset);

   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

   printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));


    // initialize the sequencer semaphores
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }

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
    timer_create(CLOCK_REALTIME, NULL, &timer_1);

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

    fclose(fp);fclose(foc_open_loop);fclose(dyno_test);
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

    //clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    //printf("Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);
    //syslog(LOG_CRIT, "Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);


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
	printf("Disabling sequencer interval timer with abort=%d and %llu of %d\n", abortTest, seqCnt, sequencePeriods);

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
    struct timespec sleepValue;

    // Set sleepValue to 2 millisecond (2e6 nanoseconds)
    sleepValue.tv_sec = 0;
    sleepValue.tv_nsec = 2e6;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    printf("S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS1) // check for synchronous abort request
    {
	// wait for service request from the sequencer, a signal handler or ISR in kernel
        sem_wait(&semS1);

        S1Cnt++;
        
        // DO WORK (50Hz state machine)
        switch(dyno.state){
            case 1: // CALIBRATION
                syslog(LOG_CRIT, "CALIBRATION:\n");
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = 0; 
                data[1] = dyno.state; // get 8 MSB
                sendCANFrame(sock, 0x123, data, 8); // send this signal only once
                dyno.state = 0; // reset the state
                motor_data.cmd_id = dyno.state; // reset the state
                FOC_motor_t_publish(lcm,"GUI",&motor_data); // send signal to GUI to reset the flag
                syslog(LOG_CRIT, "EXIT CALIBRATION:\n");
                break;
            case 2: // CONTROLLER Off
                syslog(LOG_CRIT, "CONTROLLER Off:\n");
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = 0; 
                data[1] = dyno.state;
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN FRAME 
                nanosleep(&sleepValue, NULL);
                dyno.state = 0; // reset the state
                motor_data.cmd_id = dyno.state; // reset the state
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 3: // VOLTAGE FOC
                syslog(LOG_CRIT, "VOLTAGE FOC:\n");
                dyno.vq_int = float_to_uint16(dyno.vq_cmd,-V_MAX,V_MAX);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.vq_int>>8; // get 8 MSB
                data[3] = dyno.vq_int&0xFF; // get 8 LSB
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN MESSAGE
                nanosleep(&sleepValue, NULL);
                motor_data.cmd_id = dyno.state;
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
                dyno.torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
        
                // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
                dyno.elec_power = motor_data.iq*dyno.vq_cmd;
                dyno.mech_power = dyno.torque_dyno*motor_data.velocity;
                dyno.efficiency = dyno.mech_power/dyno.elec_power;
                clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
                fprintf(foc_open_loop,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",current_realtime-start_realtime,dyno.vq_cmd,motor_data.vq,motor_data.iq,dyno.torque_dyno,motor_data.velocity,motor_data.position,dyno.elec_power,dyno.mech_power,dyno.efficiency);
                break;
            case 4: // POSITION CONTROL
                syslog(LOG_CRIT, "POSITION CONTROL:\n");
                // SENDS CAN MESSAGE
                dyno.pos_cmd_int = float_to_uint16(dyno.pos_cmd,-ONE_REV*GR,ONE_REV*GR);
                dyno.kp_int = float_to_uint8(dyno.kp,0.0,kp_max/GAIN);
                dyno.kd_int = float_to_uint8(dyno.kd,0.0,0.1/GAIN);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = dyno.state; // get 8 MSB
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.pos_cmd_int>>8; // get 8 MSB
                data[3] = dyno.pos_cmd_int&0xFF; // get 8 LSB
                data[4] = dyno.kp_int;
                data[5] = dyno.kd_int; 
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN MESSAGE
                nanosleep(&sleepValue, NULL);
                motor_data.cmd_id = dyno.state; // reset the state
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 5: // VELOCITY CONTROL
                syslog(LOG_CRIT, "VELOCITY CONTROL:\n");
                // Sends CAN message 
                dyno.vel_cmd_int = float_to_uint16(dyno.vel_cmd,-SPEED_MAX,SPEED_MAX);
                dyno.kp_int = float_to_uint8(dyno.kp,0.0,kp_max/GAIN);
                dyno.kd_int = float_to_uint8(dyno.kd,0.0,0.1/GAIN);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = 0; 
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.vel_cmd_int>>8; // get 8 MSB
                data[3] = dyno.vel_cmd_int&0xFF; // get 8 LSB
                data[4] = dyno.kp_int;
                data[5] = dyno.kd_int;
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN MESSAGE
                nanosleep(&sleepValue, NULL);
                motor_data.cmd_id = dyno.state; // reset the state
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 6: // DYNO TEST
                syslog(LOG_CRIT, "DYNO TEST:\n");

                // if total test counter is in the ramp up zone
                if(dyno.total_test_counter <= dyno.N-dyno.N_ramp_down){
                    // if cycle counter is in the ramp up zone, then increase voltage
                    if(dyno.cycle_counter <= (int)(dyno.ramp_time/dyno.h)){
                        dyno.vq_cmd = dyno.vq_cmd - dyno.dv;
                    }
                    // else keep the voltage command constant
                    else{
                        //dyno.vq_cmd = dyno.vq_cmd; // don't update command voltage
                    }
                    dyno.cycle_counter = dyno.cycle_counter + 1;
                    // if one cylce (ramp+measure) is complete, then reset cycle counter
                    if(dyno.cycle_counter > (int)(dyno.ramp_time/dyno.h) + (int)(dyno.meas_time/dyno.h)){
                        dyno.cycle_counter = 1; // reset cycle counter
                    }
                }
                // else decrease the voltage command
                else{
                    dyno.vq_cmd = dyno.vq_cmd + dyno.dv;
                }

                
                // if total test counter is complete, then reset the total test counter and rest state to zero
                if(dyno.total_test_counter >= dyno.N){
                    dyno.total_test_counter = 1; // reset total test counter
                    dyno.cycle_counter = 0; // reset cycle counter
                    dyno.state = 2; // turn controller off (this in turn will set the state to zero)
                    dyno.vq_cmd = 0.0; // reset the voltage command
                    fclose(dyno_test);

                    dyno.vq_int = float_to_uint16(dyno.vq_cmd,-V_MAX,V_MAX);
                    memset(data, 0, sizeof(data)); // Initialize data to zero
                    data[1] = dyno.state; // get 8 MSB
                    data[2] = dyno.vq_int>>8; // get 8 MSB
                    data[3] = dyno.vq_int&0xFF; // get 8 LSB
                    // un-comment to test
                    sendCANFrame(sock, 0x123, data, 8);
                    // READS CAN MESSAGE
                    nanosleep(&sleepValue, NULL);
                    motor_data.cmd_id = dyno.state;
                    // un-comment to test
                    receiveCANFrame(sock);
                    FOC_motor_t_publish(lcm, "GUI", &motor_data);
                    mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
                    dyno.torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
            
                    // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
                    dyno.elec_power = motor_data.iq*dyno.vq_cmd;
                    dyno.mech_power = dyno.torque_dyno*motor_data.velocity;
                    dyno.efficiency = dyno.mech_power/dyno.elec_power;
                    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
                    fprintf(dyno_test,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",current_realtime-start_realtime,dyno.vq_cmd,motor_data.vq,motor_data.iq,dyno.torque_dyno,motor_data.velocity,motor_data.position,dyno.elec_power,dyno.mech_power,dyno.efficiency);
                    break; // does this break the switch or the while loop?  
                }

                dyno.vq_int = float_to_uint16(dyno.vq_cmd,-V_MAX,V_MAX);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.vq_int>>8; // get 8 MSB
                data[3] = dyno.vq_int&0xFF; // get 8 LSB
                // un-comment to test
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN MESSAGE
                nanosleep(&sleepValue, NULL);
                motor_data.cmd_id = dyno.state;
                // un-comment to test
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
                dyno.torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
        
                // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
                dyno.elec_power = motor_data.iq*dyno.vq_cmd;
                dyno.mech_power = dyno.torque_dyno*motor_data.velocity;
                dyno.efficiency = dyno.mech_power/dyno.elec_power;
                clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
                fprintf(dyno_test,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",current_realtime-start_realtime,dyno.vq_cmd,motor_data.vq,motor_data.iq,dyno.torque_dyno,motor_data.velocity,motor_data.position,dyno.elec_power,dyno.mech_power,dyno.efficiency);

                dyno.total_test_counter = dyno.total_test_counter + 1; // increase total test counter

                break;
            case 7: // TOGGLE LED
                syslog(LOG_CRIT, "Toggle LED:\n");
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = dyno.led_flag; 
                dyno.state = 7;
                data[1] = dyno.state;
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN FRAME 
                nanosleep(&sleepValue, NULL);
                receiveCANFrame(sock);
                dyno.state = 0; // reset the state
                motor_data.cmd_id = dyno.state; // reset the state
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 8: // CURRENT CONTROL
                // Sends CAN message 
                dyno.iq_int = float_to_uint16(dyno.iq_cmd,-I_MAX,I_MAX);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.iq_int>>8; // get 8 MSB
                data[3] = dyno.iq_int&0xFF; // get 8 LSB
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN MESSAGE
                motor_data.cmd_id = dyno.state;
                nanosleep(&sleepValue, NULL);
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 9: // TORQUE CONTROL
                // Sends CAN message 
                dyno.torque_cmd_int = float_to_uint16(dyno.torque_cmd,-TAU_MAX,TAU_MAX);
                dyno.kp_int = float_to_uint8(dyno.kp,0.0,kp_max/GAIN);
                dyno.kd_int = float_to_uint8(dyno.kd,0.0,0.1/GAIN);
                memset(data, 0, sizeof(data)); // Initialize data to zero
                data[0] = 0; 
                data[1] = dyno.state; // get 8 MSB
                data[2] = dyno.torque_cmd_int>>8; // get 8 MSB
                data[3] = dyno.torque_cmd_int&0xFF; // get 8 LSB
                data[4] = dyno.kp_int;
                data[5] = dyno.kd_int;
                sendCANFrame(sock, 0x123, data, 8);
                // READS CAN FRAME 
                nanosleep(&sleepValue, NULL);
                motor_data.cmd_id = dyno.state; // reset the state
                receiveCANFrame(sock);
                FOC_motor_t_publish(lcm, "GUI", &motor_data);
                break;
            case 10: // ANTI-COGGING LUT
                
            default:
                // on order of up to milliseconds of latency to get time
                clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
                syslog(LOG_CRIT, "S1 50 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S1Cnt, current_realtime-start_realtime);
                //fprintf(fp, "%f,%llu\n", current_realtime-start_realtime, S1Cnt);
                break;
        }
        

	    // on order of up to milliseconds of latency to get time
        //clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        //syslog(LOG_CRIT, "S1 50 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S1Cnt, current_realtime-start_realtime);
        //fprintf(fp, "%f,%llu\n", current_realtime-start_realtime, S1Cnt);
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
    //printf("Received message on channel \"%s\":\n", channel);
    //printf("  position    = %f\n", msg->position);
    //cmd_id = msg->cmd_id;
    if(msg->cmd_id == 1)
    {
      //printf("Calibration:\n");
      dyno.state = msg->cmd_id;
    }
    else if(msg->cmd_id == 2)
    {
      //printf("Controller Off:\n");
      dyno.state = msg->cmd_id;
    }
    else if(msg->cmd_id == 3)
    {
      //printf("Voltage FOC:\n");
      dyno.state = msg->cmd_id;
      dyno.vq_cmd = msg->vq_cmd;
    }
    else if(msg->cmd_id == 4)
    {
      //printf("Position Control:\n");
      dyno.state = msg->cmd_id;
      dyno.pos_cmd = msg->position_cmd;
      dyno.kp = msg->kp;
      dyno.kd = msg->kd;
    }
    else if(msg->cmd_id == 5)
    {
      //printf("Velocity Control:\n");
      dyno.state = msg->cmd_id;
      dyno.vel_cmd = msg->velocity_cmd;
      dyno.kp = msg->kp;
      dyno.kd = msg->kd;
    }
    else if(msg->cmd_id == 6)
    {
      //printf("Dyno Test:\n");
      //dyno_test_flag = msg->dyno_test_flag;
        dyno.state = msg->cmd_id;
        dyno.vq_cmd = -dyno.dv; // always start from zero - dv
      //printf("  dyno test flag    = %d\n", msg->dyno_test_flag);
        dyno_test = fopen("dyno_test.csv", "w");
        if (dyno_test == NULL) {
            printf("Could not open file dyno_test\n");
            return 1;
        }
        fprintf(dyno_test,"Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
    
    }
    else if(msg->cmd_id == 7)
    {
      //printf("Toggle LED:\n");
      dyno.led_flag = msg->led;
      dyno.state = msg->cmd_id;
    }
    else if(msg->cmd_id == 8)
    {
      //printf("Toggle LED:\n");
      dyno.iq_cmd = msg->iq_cmd;
      dyno.state = msg->cmd_id;
    }
    else if(msg->cmd_id == 9)
    {
      //printf("Toggle LED:\n");
      dyno.torque_cmd = msg->torque_cmd;
      dyno.state = msg->cmd_id;
      dyno.kp = msg->kp;
      dyno.kd = msg->kd;
    }
      
    else
    {
      // do nothing
      //state = 0;
    }
      
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

void receiveCANFrame(int sock) {
    struct can_frame frame;
    ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("Failed to read CAN frame");
        close(sock);
        exit(1);
    }

    //printf("Received CAN frame - ID: 0x%03X, DLC: %d, Data: ", frame.can_id, frame.can_dlc);
    /*
    int i;
    for (i = 0; i < frame.can_dlc; i++) {
        printf("%02X ", frame.data[i]);
    }
    */
    
    //printf("\n");
    motor_data.position = uint16_to_float((frame.data[0]<<8 | frame.data[1]),-ONE_REV*GR,ONE_REV*GR);
    motor_data.velocity = uint16_to_float((frame.data[2]<<8 | frame.data[3]),-SPEED_MAX,SPEED_MAX);
    motor_data.vq = uint16_to_float((frame.data[4]<<8 | frame.data[5]),-V_MAX,V_MAX);
    motor_data.iq = uint16_to_float((frame.data[6]<<8 | frame.data[7]),-I_MAX,I_MAX);
    motor_data.torque = motor_data.iq*KT*GR/GR; // removing the gear ratio since testing motor without gearbox
    //FOC_motor_t_publish(lcm, "GUI", &motor_data);
}

