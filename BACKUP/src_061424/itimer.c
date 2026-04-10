/*************************************************************************************************************/
/* Function: Interval timer software "watchdog" demonstration                                                */
/*                                                                                                           */
/* Sam Siewert -  10/1/2018                                                                                  */
/*                                                                                                           */
/* References:                                                                                               */
/*                                                                                                           */
/* 1) http://mercury.pr.erau.edu/~siewerts/cec450/code/VxWorks-Examples/Basic-Feature-Examples/itimer_test.c */
/*                                                                                                           */
/*************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <lcm/lcm.h>
#include "mypackage_Position.h"
#include "FOC_motor_t.h"

#include "pmd.h"
#include "usb-1608FS.h"

#define PI 3.14159265
//#define SAMPLE_RATE 1000.0 // 100 samples per sec
#define AMPLITUDE 1.0
#define FREQUENCY 1.0
#define NANO_SEC_INTERVAL 1000000 // 1000Hz timer
#define TIMER_FREQ 1000.0 // [1Hz 1000] [10Hz = 100] [100Hz = 10]
#define ONE_HZ_TIMER 1000.0
#define TEN_HZ_TIMER 100.0
#define ONE_HUNDRED_HZ_TIMER 10.0
#define FIFTY_HZ_TIMER 20.0
#define CAN_FREQ 10.0 // 100Hz timer
#define NUM_SAMPLES 30   
static double SAMPLE_RATE = NANO_SEC_INTERVAL/TIMER_FREQ;

static long int count = 0;
static int test_count = -1; // 1Hz 
static int total_num_test_points = 0;
static int update_voltage_count = 0;
static long int loop_count = 0; // 100Hz
static long int manual_test_count = 0; // 1Hz
static long int manual_time_count = -1;
static long int vq_step_count=0;
static int vq_state = 0; 
static float vq_step = 0;
static int collect_data = 0;
static int num_test_points = 0;
static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;
void interval_expired(int id);

mypackage_Position my_data;
FOC_motor_t motor_data;
static int state = 0;
static int led_flag = 0;
static double pos_cmd = 0;
static double vel_cmd = 0;
static double vq_cmd = 0;
static double iq_cmd = 0;
static double torque_cmd = 0;
static double kp = 0;
static double kd = 0;
static int vq_int = 0;
static int iq_int = 0;
static int pos_cmd_int = 0;
static int vel_cmd_int = 0;
static int torque_cmd_int = 0;
static int kp_int = 0;
static int kd_int = 0;
static double sample_waveform;
static float torque_dyno = 0;
static float elec_power = 0;
static float mech_power = 0;
static float efficiency = 0;

lcm_t *lcm;
lcm_t *lcm2;
void *listener(void* unused);
static void my_handler(const lcm_recv_buf_t *rbuf, const char *channel, const FOC_motor_t *msg,
                       void *user);

FILE *fp;

/*Special address description flags for CAN_ID*/
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_INTERFACE_1 "vcan0"
#define CAN_INTERFACE_2 "vcan1"
int sock; // Socket descriptor
void receiveCANFrame(int sock); 
void sendCANFrame(int sock, unsigned int canId, unsigned char *data, int dataLength);
unsigned char find_can(const int port);
unsigned char data[8];

/* Variables for MCC DAQ */
static uint8_t mcc_channel = 0;
static uint8_t mcc_gain = BP_5_00V;
static int mcc_flag,mcc_ret;
static signed short mcc_svalue;
static libusb_device_handle *udev = NULL;
static Calibration_AIN table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS];
static float NEWTONS_PER_VOLTS = 0.57;//2.85/5.0;
static float GAIN = 25.0;

/* Constants needed to convert float to int */
#define I_MAX 40.0f // 40 amps max
#define V_MAX 24.0f // 24 volts max
#define TAU_MAX 2.0f // 2 Nm max
//#define KT 0.037352f // torque constant (Nm/Amps)
#define KT 0.0217f // 0.023f
#define GR 12.0f // gear ratio
#define ONE_REV 6.2831f // 2*pi radians
#define SPEED_MAX 300.0f // 300 rad/sec

int float_to_uint16(float x, float x_min, float x_max);
int float_to_uint12(float x, float x_min, float x_max);
int float_to_uint8(float x, float x_min, float x_max);

float uint16_to_float(int x_int, float x_min, float x_max);
float uint12_to_float(int x_int, float x_min, float x_max);
float uint8_to_float(int x_int, float x_min, float x_max);


int main() 
{
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

  // ========= END OF MCC CODE ==========

  int i, flags = 0;
  
  // Create an LCM instance
  lcm = lcm_create(NULL);
  if (!lcm)
      return 1;
  
  lcm2 = lcm_create(NULL);
  if (!lcm2)
      return 1;
    
  my_data.x = 15;
  my_data.y = 11;
  
  FOC_motor_t_subscribe(lcm2, "MOTOR", &my_handler, NULL);
  pthread_t th1;
  // Start a second thread to listen for LCM messages
  pthread_create(&th1, NULL, listener, NULL); 

  /* set up to signal SIGALRM if timer expires */
  timer_create(CLOCK_REALTIME, NULL, &timer_1);

  signal(SIGALRM, (void(*)()) interval_expired);

  /* arm the wd timer */
  itime.it_interval.tv_sec = 0;
  itime.it_interval.tv_nsec = NANO_SEC_INTERVAL;
  itime.it_value.tv_sec = 0;
  itime.it_value.tv_nsec = NANO_SEC_INTERVAL;

  timer_settime(timer_1, flags, &itime, &last_itime);

  // ------------------
  // CAN2USB
    
  // ** VIRTUAL CAN **
  //system("sudo ifconfig vcan0 down");
  //system("sudo ifconfig vcan1 down");
  /*
  system("sudo ip link add vcan0 type vcan");
  system("sudo ip link set up vcan0");
  system("sudo ip link add vcan1 type vcan");
  system("sudo ip link set up vcan1");
  system("sudo modprobe can-gw");
  system("sudo cangw -A -s vcan0 -d vcan1 -e ");
  system("sudo cangw -A -s vcan1 -d vcan0 -e");
  */

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
		printf("Transmit extended frame!\n");
    printf("can_id  = 0x%X\r\n", frame.can_id);
    printf("can_dlc = %d\r\n", frame.can_dlc);

  // prints data frame
  for(i = 0; i < frame.can_dlc; i++)
      printf("data[%d] = %d\r\n", i, frame.data[i]);

  // Send a message from sock1 to sock2
  sendCANFrame(sock, 0x123, data, 6);

  // Receive the message on sock2
  receiveCANFrame(sock);

  //--------------------
  

  for(;;)
  {
    pause();
  }

  close(sock);
  printf("exiting from main program\n");
  pthread_join(th1, NULL);  
  // Cleanup and exit
  lcm_destroy(lcm);
  lcm_destroy(lcm2);
    
  return 0;

}

void interval_expired(int id) // 1000 Hz timer 
{
  int flags = 0;
  struct timespec rtclock_time;

  clock_gettime(CLOCK_REALTIME, &rtclock_time);
  count++;

  sample_waveform = AMPLITUDE*sin(2.0 * PI * FREQUENCY / SAMPLE_RATE * count);
  //mypackage_Position_publish(lcm, "EXAMPLE", &my_data);
  if(state == 1) // calibration
  {
    printf("Calibration:\n");
    memset(data, 0, sizeof(data)); // Initialize data to zero
    data[0] = 0; 
    data[1] = state; // get 8 MSB
    sendCANFrame(sock, 0x123, data, 8); // send this signal only once
    state = 0; // reset the state
    motor_data.cmd_id = state; // reset the state
    FOC_motor_t_publish(lcm,"GUI",&motor_data); // send signal to GUI to reset the flag
    printf("Exit Calibration:\n");
  }
  else if(state == 2) // Controller off
  {
    printf("Controller Off:\n");
    //frame.data[0] = state; // get 8 MSB
    //write(s, &frame, sizeof(struct can_frame));
    memset(data, 0, sizeof(data)); // Initialize data to zero
    data[0] = 0; 
    data[1] = state;
    sendCANFrame(sock, 0x123, data, 8);
    // READS CAN FRAME 
    state = 0; // reset the state
    motor_data.cmd_id = state; // reset the state
    receiveCANFrame(sock);
    //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
    FOC_motor_t_publish(lcm, "GUI", &motor_data);

    // added this for manual data collection on 10/08/23
    test_count = -1;
    manual_test_count = -1;
    manual_time_count = -1;
    if(collect_data == 1)
    {
      collect_data = 0;
      fclose(fp);
      printf(" close csv file\n");
    }

  }
  else if(state == 3) // 100Hz voltage FOC
  {
    
    vq_step_count++;
    if(vq_step_count > ONE_HZ_TIMER)
    {
      if(vq_state == 0)
      {
        vq_step = 0.3; // turn it on
        vq_state = 1;
      }
      else
      {
        vq_step = 0.0;
        vq_state = 0;
      }
      vq_step_count = 0;
    }
    
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      //printf("Vq cmd: %f\n",vq_cmd);
      // Sends CAN message 
      //vq_int = float_to_uint16(vq_step,-V_MAX,V_MAX);
      vq_int = float_to_uint16(vq_cmd,-V_MAX,V_MAX);
      memset(data, 0, sizeof(data)); // Initialize data to zero
      data[1] = state; // get 8 MSB
      data[2] = vq_int>>8; // get 8 MSB
      data[3] = vq_int&0xFF; // get 8 LSB
      sendCANFrame(sock, 0x123, data, 8);
      // READS CAN MESSAGE
      motor_data.cmd_id = state;
      receiveCANFrame(sock);
      //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
      FOC_motor_t_publish(lcm, "GUI", &motor_data);
    }

    // ========================
    // added this section for manual data collection on 10/08/23
    test_count++; 
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      collect_data = 1;
      fp = fopen("data_v2.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    manual_test_count++; // commented this on 12/02/23. It was working fine before commenting this out fyi
    manual_time_count++;
    if(manual_test_count > ONE_HZ_TIMER/100.0) // sample at ([ONE_HZ_TIMER/10.0] for 10Hz) ([ONE_HZ_TIMER/50.0] for 50Hz) ([ONE_HZ_TIMER/100.0] for 100Hz)
    {
      manual_test_count = -1;
      mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
      torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
      

      // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
      elec_power = motor_data.iq*vq_cmd;
      mech_power = torque_dyno*motor_data.velocity;
      efficiency = mech_power/elec_power;
      fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(vq_cmd),motor_data.vq,(motor_data.iq),(torque_dyno),(motor_data.velocity),motor_data.position,(elec_power),(mech_power),(efficiency));
      fflush(fp);
    }
    // ===================

  }
  else if(state == 4) // 100Hz Position controller
  {    
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      //printf("Position,kp,kd: %f,%f,%f\n",pos_cmd,kp,kd);
      // Sends CAN message 
      pos_cmd_int = float_to_uint16(pos_cmd,-ONE_REV*GR,ONE_REV*GR);
      kp_int = float_to_uint8(kp,0.0,0.1/GAIN);
      kd_int = float_to_uint8(kd,0.0,0.1/GAIN);
      memset(data, 0, sizeof(data)); // Initialize data to zero
      data[0] = state; // get 8 MSB
      data[1] = state; // get 8 MSB
      data[2] = pos_cmd_int>>8; // get 8 MSB
      data[3] = pos_cmd_int&0xFF; // get 8 LSB
      data[4] = kp_int;
      data[5] = kd_int; 
      sendCANFrame(sock, 0x123, data, 8);
      // READS CAN MESSAGE
      motor_data.cmd_id = state; // reset the state
      receiveCANFrame(sock);
      //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
      FOC_motor_t_publish(lcm, "GUI", &motor_data);
    }
  }
  else if(state == 5) // 100Hz Velocity controller
  {
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      //printf("Velocity,kp,kd: %f,%f,%f\n",vel_cmd,kp,kd);
      // Sends CAN message 
      vel_cmd_int = float_to_uint16(vel_cmd,-SPEED_MAX,SPEED_MAX);
      kp_int = float_to_uint8(kp,0.0,0.1/GAIN);
      kd_int = float_to_uint8(kd,0.0,0.1/GAIN);
      memset(data, 0, sizeof(data)); // Initialize data to zero
      data[0] = 0; 
      data[1] = state; // get 8 MSB
      data[2] = vel_cmd_int>>8; // get 8 MSB
      data[3] = vel_cmd_int&0xFF; // get 8 LSB
      data[4] = kp_int;
      data[5] = kd_int;
      sendCANFrame(sock, 0x123, data, 8);
      
      motor_data.cmd_id = state; // reset the state
      receiveCANFrame(sock);
      //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
      FOC_motor_t_publish(lcm, "GUI", &motor_data);
    }

    // ========================
    // added this section for manual data collection on 10/08/23
    test_count++; 
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      collect_data = 1;
      fp = fopen("data_v2.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage (V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    manual_test_count++;
    manual_time_count++;
    if(manual_test_count > ONE_HZ_TIMER) // sample at 1Hz
    {
      manual_test_count = -1;
      mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
      torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
      

      // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
      elec_power = motor_data.iq*motor_data.vq;
      mech_power = torque_dyno*motor_data.velocity;
      efficiency = mech_power/elec_power;
      fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),(motor_data.iq),(torque_dyno),(motor_data.velocity),(motor_data.position),(elec_power),(mech_power),(efficiency));
      fflush(fp);
    }
    // ===================

  }
  else if(state == 6) // dyno test
  {
    test_count++; 
    update_voltage_count++;
    num_test_points++;
    manual_time_count++;
    
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      fp = fopen("data.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage CMD(V),Voltage MSR(V),Current (A),Torque (Nm),Speed (rad/s),Pos (rad),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    // else if(test_count == 1) // open the file at the target frequency
    // {
    //   /*
    //   fp = fopen("data.csv","a");
    //   if(fp == NULL)
    //   {
    //     printf("Error opening file!\n");
    //     exit(1);
    //   }
    //   */
    // }
    // sampling frequency is 100Hz
    if(test_count > ONE_HUNDRED_HZ_TIMER) // 1Hz #define TIMER_FREQ 1000.0 // [1Hz = 1000] [10Hz = 100] [100Hz = 10]
    {
      if(num_test_points > 3*ONE_HZ_TIMER)
      {
        total_num_test_points = total_num_test_points + 1;
        num_test_points = 0;
      }
      
      test_count = 0; // reset the counter 
      // if true, then terminate the dyno test script
      if(total_num_test_points > NUM_SAMPLES) // if true, terminate dyno test
      {
        test_count = -1;
        total_num_test_points = 0;
        motor_data.vq_cmd = 0;
        state = 0; // reset the state
        motor_data.cmd_id = state; // reset the state
        //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag    
        
        // Sends CAN message 
        vq_int = float_to_uint16(motor_data.vq_cmd,-V_MAX,V_MAX);
        memset(data, 0, sizeof(data)); // Initialize data to zero
        data[0] = 0; 
        data[1] = 6; // get 8 MSB
        data[2] = vq_int>>8; // get 8 MSB
        data[3] = vq_int&0xFF; // get 8 LSB
        // !!!!!!!!!!!!!!!!
        sendCANFrame(sock, 0x123, data, 8); // UNCOMMENT WHEN CONNECTED TO MOTOR
        // !!!!!!!!!!!!!!!!
        receiveCANFrame(sock); // UNCOMMENT WHEN CONNECTED TO MOTOR
        FOC_motor_t_publish(lcm, "GUI", &motor_data);


      }
      else // else continue sweeping voltage inputs
      {
        // if ramp true then ramp up voltage linearly for one second
        if(update_voltage_count < ONE_HZ_TIMER)
        {
          motor_data.vq_cmd += 0.001; // 
          printf("  vq_cmd    = %f\n", motor_data.vq_cmd);
          //update_voltage_count = 0;
        }
        // collect data at steady state for two seconds
        else if(update_voltage_count >= ONE_HZ_TIMER && update_voltage_count < 3*ONE_HZ_TIMER)
        {
          //motor_data.vq_cmd += 0.1; // 
          //printf("  vq_cmd    = %f\n", motor_data.vq_cmd);
          mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
          torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
          elec_power = motor_data.iq*motor_data.vq;
          mech_power = torque_dyno*motor_data.velocity;
          efficiency = mech_power/elec_power;
          fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),motor_data.vq,(motor_data.iq),(torque_dyno),(motor_data.velocity),motor_data.position,(elec_power),(mech_power),(efficiency));
          fflush(fp);
          
        }
        else
        {
          update_voltage_count = 0;
        }
        // else hold the voltage for one second


        // Update open-loop vq voltage every second
        // if(update_voltage_count > ONE_HZ_TIMER)
        // {
        //   motor_data.vq_cmd += 0.1; // 
        //   printf("  vq_cmd    = %f\n", motor_data.vq_cmd);
        //   update_voltage_count = 0;
        // }
        // else don't update the vq_cmd

        // Sends CAN message 
        vq_int = float_to_uint16(motor_data.vq_cmd,-V_MAX,V_MAX);
        memset(data, 0, sizeof(data)); // Initialize data to zero
        data[0] = 0; 
        data[1] = state; // get 8 MSB
        data[2] = vq_int>>8; // get 8 MSB
        data[3] = vq_int&0xFF; // get 8 LSB
        // !!!!!!!!!!!!!!!!11
        sendCANFrame(sock, 0x123, data,8); // UNCOMMENT WHEN CONNECTED TO MOTOR
        // READS CAN MESSAGE
        motor_data.cmd_id = state; // reset the state
        receiveCANFrame(sock); // UNCOMMENT WHEN CONNECTED TO MOTOR
        FOC_motor_t_publish(lcm, "GUI", &motor_data);
      }
      // Sample at 100Hz

      if(total_num_test_points > NUM_SAMPLES)
      {
        fclose(fp);
        printf(" close csv file\n");
      }
      else
      {
        // mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
        // torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
        // elec_power = motor_data.iq*motor_data.vq;
        // mech_power = torque_dyno*motor_data.velocity;
        // efficiency = mech_power/elec_power;
        // fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),motor_data.vq,(motor_data.iq),(torque_dyno),(motor_data.velocity),motor_data.position,(elec_power),(mech_power),(efficiency));
        // fflush(fp);
      }
    }
  }
  else if(state == 7) // Toggle LED
  {
    printf("Toggle LED:\n");
    //frame.data[0] = state; // get 8 MSB
    //write(s, &frame, sizeof(struct can_frame));
    memset(data, 0, sizeof(data)); // Initialize data to zero
    data[0] = led_flag; 
    state = 7;
    data[1] = state;
    sendCANFrame(sock, 0x123, data, 8);
    // READS CAN FRAME 
    state = 0; // reset the state
    motor_data.cmd_id = state; // reset the state
    //for(int i=0;i<1000;i++){} // Delay
    receiveCANFrame(sock);
    //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
    FOC_motor_t_publish(lcm, "GUI", &motor_data);
  }
  else if(state == 8) // 100Hz Current Controller
  {
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      //printf("Vq cmd: %f\n",vq_cmd);
      // Sends CAN message 
      iq_int = float_to_uint16(iq_cmd,-I_MAX,I_MAX);
      memset(data, 0, sizeof(data)); // Initialize data to zero
      data[1] = state; // get 8 MSB
      data[2] = iq_int>>8; // get 8 MSB
      data[3] = iq_int&0xFF; // get 8 LSB
      sendCANFrame(sock, 0x123, data, 8);
      // READS CAN MESSAGE
      motor_data.cmd_id = state;
      receiveCANFrame(sock);
      //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
      FOC_motor_t_publish(lcm, "GUI", &motor_data);
    }
    
    // ========================
    // added this section for manual data collection on 10/08/23
    test_count++; 
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      collect_data = 1;
      fp = fopen("data_v3.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage (V),Current CMD(A),Current MSR (A),Torque (Nm),Speed (rad/s),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    manual_test_count++;
    manual_time_count++;
    if(manual_test_count > ONE_HZ_TIMER/100.0) // sample at 1Hz
    {
      manual_test_count = -1;
      mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
      torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.82Nm/5V
      

      // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
      elec_power = iq_cmd*motor_data.vq;
      mech_power = torque_dyno*motor_data.velocity;
      efficiency = mech_power/elec_power;
      fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),(iq_cmd),motor_data.iq,(torque_dyno),(motor_data.velocity),(elec_power),(mech_power),(efficiency));
      fflush(fp);
    }
    // ===================
    

  }
  else if(state == 9) // 100Hz Torque controller
  {
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      //printf("Velocity,kp,kd: %f,%f,%f\n",vel_cmd,kp,kd);
      // Sends CAN message 
      torque_cmd_int = float_to_uint16(torque_cmd,-TAU_MAX,TAU_MAX);
      kp_int = float_to_uint8(kp,0.0,0.1/GAIN);
      kd_int = float_to_uint8(kd,0.0,0.1/GAIN);
      memset(data, 0, sizeof(data)); // Initialize data to zero
      data[0] = 0; 
      data[1] = state; // get 8 MSB
      data[2] = torque_cmd_int>>8; // get 8 MSB
      data[3] = torque_cmd_int&0xFF; // get 8 LSB
      data[4] = kp_int;
      data[5] = kd_int;
      sendCANFrame(sock, 0x123, data, 8);
      
      motor_data.cmd_id = state; // reset the state
      receiveCANFrame(sock);
      //FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
      FOC_motor_t_publish(lcm, "GUI", &motor_data);
    }

    // ========================
    // added this section for manual data collection on 10/08/23
    test_count++; 
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      collect_data = 1;
      fp = fopen("data_v4.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage (V),Current (A),Torque MSR (Nm),Torque CMD (Nm),Speed (rad/s),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    manual_test_count++;
    manual_time_count++;
    if(manual_test_count > ONE_HZ_TIMER) // sample at 1Hz
    {
      manual_test_count = -1;
      mcc_svalue = usbAIn_USB1608FS(udev, mcc_channel, mcc_gain, table_AIN);
      torque_dyno = volts_USB1608FS(mcc_gain, mcc_svalue)*NEWTONS_PER_VOLTS; // 2.85Nm/5V
      

      // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
      elec_power = motor_data.iq*motor_data.vq;
      mech_power = torque_dyno*motor_data.velocity;
      efficiency = mech_power/elec_power;
      fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f\n",(double)manual_time_count,(motor_data.vq),(motor_data.iq),(torque_dyno),torque_cmd,(motor_data.velocity),(elec_power),(mech_power),(efficiency));
      fflush(fp);
    }
    // ===================

  }
  else // Do nothing
  {
    // do nothing
    //printf("State: %d\n", state);
  }
  //printf("State: %d\n", state);
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
      state = msg->cmd_id;
    }
    else if(msg->cmd_id == 2)
    {
      //printf("Controller Off:\n");
      state = msg->cmd_id;
    }
    else if(msg->cmd_id == 3)
    {
      //printf("Voltage FOC:\n");
      state = msg->cmd_id;
      vq_cmd = msg->vq_cmd;
    }
    else if(msg->cmd_id == 4)
    {
      //printf("Position Control:\n");
      state = msg->cmd_id;
      pos_cmd = msg->position_cmd;
      kp = msg->kp;
      kd = msg->kd;
    }
    else if(msg->cmd_id == 5)
    {
      //printf("Velocity Control:\n");
      state = msg->cmd_id;
      vel_cmd = msg->velocity_cmd;
      kp = msg->kp;
      kd = msg->kd;
    }
    else if(msg->cmd_id == 6)
    {
      printf("Dyno Test:\n");
      //dyno_test_flag = msg->dyno_test_flag;
      state = msg->cmd_id;
      //printf("  dyno test flag    = %d\n", msg->dyno_test_flag);
    }
    else if(msg->cmd_id == 7)
    {
      //printf("Toggle LED:\n");
      led_flag = msg->led;
      state = msg->cmd_id;
    }
    else if(msg->cmd_id == 8)
    {
      //printf("Toggle LED:\n");
      iq_cmd = msg->iq_cmd;
      state = msg->cmd_id;
    }
    else if(msg->cmd_id == 9)
    {
      //printf("Toggle LED:\n");
      torque_cmd = msg->torque_cmd;
      state = msg->cmd_id;
      kp = msg->kp;
      kd = msg->kd;
    }
      
    else
    {
      // do nothing
      //state = 0;
    }
      
}

// =========================================================

int float_to_uint4(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<4)-1))/span);
    //return ((int) (x-offset)*((float)((1<<16)-1))/span);
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

float uint4_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<4)-1)) + offset;
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

