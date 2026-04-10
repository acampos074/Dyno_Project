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
#include <errno.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <lcm/lcm.h>
#include "mypackage_Position.h"
#include "FOC_motor_t.h"

#define PI 3.14159265
//#define SAMPLE_RATE 1000.0 // 100 samples per sec
#define AMPLITUDE 2.0
#define FREQUENCY 4.0
#define NANO_SEC_INTERVAL 1000000 // 1000Hz timer
#define TIMER_FREQ 1000.0 // 1Hz timer
#define CAN_FREQ 10.0 // 100Hz timer
#define NUM_SAMPLES 10   

static long int count = 0;
static int test_count = -1; // 1Hz 
static long int loop_count = 0; // 100Hz
static int num_test_points = 0;

static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;

mypackage_Position my_data;
FOC_motor_t motor_data;
static int state = 0;
static double pos_cmd = 0;
static double vel_cmd = 0;
static double vq_cmd = 0;
static double kp = 0;
static double kd = 0;

lcm_t *lcm;
lcm_t *lcm2;
static double SAMPLE_RATE = NANO_SEC_INTERVAL/TIMER_FREQ;

FILE *fp;

void interval_expired(int id) // 1000 Hz timer 
{
  int flags = 0;
  struct timespec rtclock_time;

  clock_gettime(CLOCK_REALTIME, &rtclock_time);
  count++;

  my_data.x = AMPLITUDE*sin(2.0 * PI * FREQUENCY / SAMPLE_RATE * count);
  mypackage_Position_publish(lcm, "EXAMPLE", &my_data);

  if(state == 1)
  {
    printf("Calibration:\n");
    state = 0; // reset the state
    motor_data.cmd_id = state; // reset the state
    FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
  }
  else if(state == 2)
  {
    printf("Controller Off:\n");
    state = 0; // reset the state
    motor_data.cmd_id = state; // reset the state
    FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
  }
  else if(state == 3) // 100Hz
  {
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      printf("Vq cmd: %f\n",vq_cmd);
    }
  }
  else if(state == 4) // 100Hz
  {    
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      printf("Position,kp,kd: %f,%f,%f\n",pos_cmd,kp,kd);
    }
  }
  else if(state == 5) // 100Hz
  {
    loop_count++;
    if(loop_count > CAN_FREQ)
    {
      loop_count = 0;
      printf("Velocity,kp,kd: %f,%f,%f\n",vel_cmd,kp,kd);
    }
  }
  else if(state == 6) // dyno test
  {
    test_count++; 
    //open a csv file to write data
    if(test_count == 0) // open the file only once
    {
      
      fp = fopen("data.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
      fprintf(fp,"Time (s),Voltage (V),Current (A),Torque (Nm),Speed (rad/s),Elec Power (W),Mech Power (W),Efficiency\n");
      fflush(fp);
      printf(" open csv file\n");
    }
    else if(test_count == 1) // open the file at the target frequency
    {
      
      fp = fopen("data.csv","a");
      if(fp == NULL)
      {
        printf("Error opening file!\n");
        exit(1);
      }
    }
    if(test_count > TIMER_FREQ) // 1Hz 
    {
      test_count = 0;
      num_test_points = num_test_points + 1;
      if(num_test_points > NUM_SAMPLES) // sweep across 10 cmd voltages
      {
        num_test_points = 0;
        motor_data.vq_cmd = 0;
        state = 0; // reset the state
        motor_data.cmd_id = state; // reset the state
        FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag    
        fclose(fp);
        printf(" close csv file\n");
        // TODO: send CAN message 
      }
      else
      {
        // Send updated open-loop vq voltage
        motor_data.vq_cmd += 2; // turn-off dyno test
        printf("  vq_cmd    = %f\n", motor_data.vq_cmd);
        // [time,vq,iq,tau,speed,electrical power,mech power,efficiency]
        fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,%f\n",(double)num_test_points,motor_data.vq_cmd,3.0,4.0,5.0,6.0,7.0,8.0);
        fflush(fp);
        FOC_motor_t_publish(lcm2,"MOTOR",&motor_data); // send signal to GUI to reset the flag
        // TODO: send CAN message 
      }
    }
  }
  else
  {
    // do nothing
  }
}

// TODO: integrate ADC code to collect data
// TODO: integrate CAN2USB code to communicate GUI signals to FOC board

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
      
    else
    {
      // do nothing
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


int main() 
{

  int i;
  int flags = 0;
  
  // Create an LCM instance
  //lcm_t *lcm = lcm_create(NULL);
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

  for(;;)
  {
    pause();
  }
    
  printf("exiting from main program\n");
  pthread_join(th1, NULL);  
  // Cleanup and exit
  lcm_destroy(lcm);
  lcm_destroy(lcm2);
    
  return 0;

}

