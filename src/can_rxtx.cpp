/*This is a socket CAN transmiter test programmer*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <chrono>
#include <thread>
#include <iostream>
//#include <hiredis/hiredis.h>
#include "/home/campo074/Documents/mujoco200_linux/include/RedisClient.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>



/*Special address description flags for CAN_ID*/
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
unsigned char  find_can(const int port)
{
 char buf[128]={0};
 sprintf(buf,"/sys/class/net/can%d/can_bittiming/bitrate",port);
 return((access(buf,0)==0));
}

int float_to_uint16(float x, float x_min, float x_max);
int float_to_uint8(float x, float x_min, float x_max);
float uint16_to_float(int x_int, float x_min, float x_max);
float uint8_to_float(int x_int, float x_min, float x_max);

const std::string JOINT_ANGLES_KEY  = "q";
const std::string JOINT_VELOCITIES_KEY = "dq";
const std::string JOINT_TORQUES_KEY = "u";
const std::string JOINT_TORQUES_CMD_KEY = "tau";
const std::string CONTROL_FLAG_KEY  = "flag";
const std::string LED_KEY = "led";
const std::string GENERAL_COMMAND_KEY = "cmd";
const std::string JOINT_VELOCITY_CMD_KEY = "vel_cmd";
const std::string KP_KEY = "kp";
const std::string KD_KEY = "kd";


/* Constants needed to convert float to int */
#define I_MAX 40.0f // 40 amps max
#define KT 0.0249f // torque constant (Nm/Amps)
#define GR 12.0f // gear ratio
#define ONE_REV 6.2831f // 2*pi radians
#define MAX_SPEED 65.0f // 65 rad/sec

std::string ctl_flag;
std::string led_flag;
static std::string gen_cmd;
//static std::string dq_cmd;

int main()
{
    // Make sure redis-server is running at localhost with default port 6379
	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

    Eigen::VectorXd v_q(1);
    Eigen::VectorXd v_dq(1);
    Eigen::VectorXd v_u(1);
    Eigen::VectorXd v_tau_cmd(1);
    Eigen::VectorXd dq_cmd(1);
    Eigen::VectorXd v_kp(1);
    Eigen::VectorXd v_kd(1);
    int torque_int = 0;
    int angle_int = 0;
    int velocity_int = 0;
    int torque_cmd_int = 0;
    int velocity_cmd_int = 0;
    int kp_int = 0;
    int kd_int = 0;
    float kp_float = 0;

    v_q << 0;
    v_dq << 0;
    v_u << 0;
    v_tau_cmd << 0;
    dq_cmd << 0;
    v_kp << 0;
    v_kd << 0;
    redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,v_q);	 
    redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY,v_dq);
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_KEY,v_u);
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_CMD_KEY,v_tau_cmd);
    redis_client.setEigenMatrixDerivedString(KP_KEY,v_kp);
    redis_client.setEigenMatrixDerivedString(KD_KEY,v_kd);
    redis_client.setCommandIs(CONTROL_FLAG_KEY,"0");
    redis_client.setCommandIs(GENERAL_COMMAND_KEY,"0");
    //redis_client.setCommandIs(JOINT_VELOCITY_CMD_KEY,"0");
    redis_client.setEigenMatrixDerivedString(JOINT_VELOCITY_CMD_KEY,dq_cmd);

    //using namespace std::literals::chrono_literals;

    int ret;
    int s, nbytes,i;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    
  /* if(!find_can(0))
  	  {
	   printf("Can0 device dose not exist!\n ");
	   return -1;
	  }
*/
    system("sudo ifconfig can0 down");// must close can device before set baud rate!
    //below mean depend on iprout tools ,not ip tool with busybox!
    //system("sudo ifconfig can0 txqueuelen 1000"); // added this line to see if this fixes the incomplete mssg issue
	system("sudo ip link set can0 type can bitrate 1000000");
    
    //system("sudo echo 1000000 > /sys/class/net/can0/can_bittiming/bitrate");
    system("sudo ifconfig can0 up");
    printf("This is a socket can transmit & receive demo program ,can0 with 1Mbps baud rate\r\n");
        
    /*Create socket*/
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Create socket PF_CAN failed!");
        return 1;
    }
    
    /*Specify can0 device*/
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl interface index failed!");
        return 1;
    }
    
    /*Bind the socket to can0*/
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed!");
        return 1;
    }
    
    /*Disable filtering rules,this program only send message do not receive packets */
    //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    /*Define receive filter rules,we can set more than one filter rule!*/
    struct can_filter rfilter[2];
    rfilter[0].can_id = 0x123;//Standard frame id !
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x12345678;//extend frame id!
    rfilter[1].can_mask = CAN_EFF_MASK;

    /*assembly  message data! */
    frame.can_id = 0x123;
    frame.can_dlc = 6; // Using 2 bytes to encode a single torque value,1 for led flag,1 for general command, 1 for speed cmd. Max of 8
    frame.data[0] = 0x12;
    frame.data[1] = 0x34;
    frame.data[2] = 3;
    frame.data[3] = 4;
    frame.data[4] = 5;
    frame.data[5] = 6;
    //frame.data[6] = 7;
    //frame.data[7] = 8;
    

    //if(frame.can_id&CAN_EFF_FLAG==0)
    if(!(frame.can_id&CAN_EFF_FLAG))
		printf("Transmit standard frame!\n");
	else
		printf("Transmit extended frame!\n");
        printf("can_id  = 0x%X\r\n", frame.can_id);
        printf("can_dlc = %d\r\n", frame.can_dlc);

    // prints data frame
    for(i = 0; i < frame.can_dlc; i++)
        printf("data[%d] = %d\r\n", i, frame.data[i]);
    
    // Timer variables
    //clock_t now = clock();
    //double delay = 0.0005; // time in seconds (0.001s for 1kHz timer)
    //delay *= CLOCKS_PER_SEC;
    //double delay = 0.9; // ms
    double delay = 0.25; // ms 2khz
    //double delay = 0.1; // ms >2khz
    //double delay = 1; // ms >200hz
    //double delay = 10; // ms >200/3 hz
    int counts = 0;
    //std::chrono::duration<float> duration;
    auto start = std::chrono::high_resolution_clock::now();
    auto previous = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span;
    
    // Impedance controller frequency
    while(counts < 1000000) 
    //while(1) 
    {
        now = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(now-previous);
        //std::cout << (time_span).count()*1000.0f << "ms\n";
        //if(clock() - now > delay)
        if( (time_span).count()*1000.0 > delay )
        //if(1)
        { 
            //ctl_flag = redis_client.get(CONTROL_FLAG_KEY);
            bool reply = redis_client.getCommandIs(CONTROL_FLAG_KEY,ctl_flag);
            bool reply2 = redis_client.getCommandIs(LED_KEY,led_flag);
            bool reply3 = redis_client.getCommandIs(GENERAL_COMMAND_KEY,gen_cmd);

            if(std::stoi(ctl_flag) == 1)
            {
                // Read Redis keys and send them to the MCU (target torque values)
                redis_client.getEigenMatrixDerivedString(JOINT_TORQUES_KEY,v_u);
                // Convert double to int and package data into 2 byte packets
                torque_int = float_to_uint16(v_u[0],-I_MAX*KT*GR,I_MAX*KT*GR);
                // Packages the torque value in two bytes
                redis_client.getEigenMatrixDerivedString(JOINT_VELOCITY_CMD_KEY,dq_cmd);
                velocity_cmd_int = float_to_uint16(dq_cmd[0],-65.0f,65.0f);
                redis_client.getEigenMatrixDerivedString(KP_KEY,v_kp);
                kp_int = float_to_uint8(v_kp[0],0.0f,0.1f);
                redis_client.getEigenMatrixDerivedString(KD_KEY,v_kd);
                kd_int = float_to_uint8(v_kd[0],0.0f,0.1f);
                //printf("kp: %d \r\n",kp_int);
                //printf("kd: %d \r\n",kd_int);
                //frame.data[0] = torque_int>>8;    // get 8 MSB
                //frame.data[1] = torque_int&0xFF;  // get 8 LSB
                frame.data[0] = std::stoi(led_flag);
                frame.data[1] = std::stoi(gen_cmd);
                frame.data[2] = velocity_cmd_int>>8;
                frame.data[3] = velocity_cmd_int&0xFF;
                // Can only Tx 6 bytes since STM32 has a hardware issue where it can only 
                // RX up to 6 bytes
                frame.data[4] = kp_int;
                frame.data[5] = kd_int;
                //printf("%u \r\n",frame.data[6]);
                kp_float = uint8_to_float(kp_int,0.0f,0.1f);
                //printf("%f \r\n",kp_float);
                //frame.data[0] = 0x12;
                //frame.data[1] = 0x34;

                /*Tx data to trigger MCU interrupt */
                //nbytes = write(s, &frame, 64);

                nbytes = write(s, &frame, sizeof(frame)); 
                //printf("%ld \r\n",sizeof(frame));
                
                if(nbytes != sizeof(frame)) 
                {
                    printf("Send  frame incompletely!\r\n");
                    system("sudo ifconfig can0 down");
                }
                
                

                /* Rx data */
                nbytes = read(s, &frame, sizeof(frame));
                //printf("%d \r\n",nbytes);
                if(nbytes > 0) 
                {
                    if(!(frame.can_id&CAN_EFF_FLAG))
                    {
                        //if(frame.can_id&0x80000000==0)
                        //printf("Received standard frame!\n");
                    }

                    else 
                    {
                        printf("Received extended frame!\n");
                    }
                        
                    //printf("can_id = 0x%X\r\ncan_dlc = %d \r\n", frame.can_id&0x1FFFFFFF, frame.can_dlc);
                    for(i = 0; i < 6; i++) // expected three receive four bytes (2 for position & 2 for velocity & 2 for torque cmd)
                    {
                        //printf("data[%d] = %d\r\n", i, frame.data[i]);
                        
                    }
                    
                    
                    // Read packaged data
                    angle_int = (frame.data[0]<<8 | frame.data[1]);
                    velocity_int = (frame.data[2]<<8 | frame.data[3]);
                    torque_cmd_int = (frame.data[4]<<8 | frame.data[5]);
                    // Convert int to double 
                    v_q << uint16_to_float(angle_int,-ONE_REV*GR,ONE_REV*GR);
                    v_dq << uint16_to_float(velocity_int,-MAX_SPEED,MAX_SPEED);
                    v_tau_cmd << uint16_to_float(torque_cmd_int,-I_MAX,I_MAX);
                    //v_tau_cmd << uint16_to_float(angle_int,-ONE_REV*GR,ONE_REV*GR); // for debugging
                    // Update Redis Keys
                    redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY,v_q);
                    redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY,v_dq);
                    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_CMD_KEY,v_tau_cmd);
                    //redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_CMD_KEY,v_q); // for debugging
                        // mask below sentense to receive all the time other wise can only receive one time!
                        // break;
                }

                //now = clock();
                /*
                end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(end-previous);
                std::cout << (dt).count()*1000.0f << "ms\n";
                */
                previous = std::chrono::high_resolution_clock::now();
                counts++;
                // Reset flag to zero
                ctl_flag = "0";
                redis_client.setCommandIs(CONTROL_FLAG_KEY,"0");
            }  
            
        }
    }
    end = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end-start);
    std::cout << (time_span).count()*1000.0f << "ms\n";
    /*Close the socket and can0 */
    close(s);
    system("sudo ifconfig can0 down");
    return 0;
}

int float_to_uint8(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<8)-1))/span);
    //return ((int) (x-offset)*((float)((1<<16)-1))/span);
}

int float_to_uint16(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<16)-1))/span);
    //return ((int) (x-offset)*((float)((1<<16)-1))/span);
}

float uint16_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<16)-1)) + offset;
}

float uint8_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<8)-1)) + offset;
}