#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_driver.h"
#include "encoding.h"
#include "dyno.h"

static int can_sock = -1;

int can_open(void)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    system("sudo ifconfig can0 down");
    system("sudo ip link set can0 type can bitrate 1000000");
    system("sudo ifconfig can0 up");
    printf("CAN: can0 configured at 1 Mbps\n");

    can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock == -1) {
        perror("can_open: socket");
        return -1;
    }

    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(can_sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("can_open: ioctl");
        close(can_sock);
        can_sock = -1;
        return -1;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
        perror("can_open: bind");
        close(can_sock);
        can_sock = -1;
        return -1;
    }

    printf("CAN: socket bound to %s\n", CAN_INTERFACE);
    return 0;
}

void can_close(void)
{
    if (can_sock >= 0) {
        close(can_sock);
        can_sock = -1;
    }
}

void sendCANFrame(unsigned int canId, unsigned char *data, int dataLength)
{
    struct can_frame frame;
    frame.can_id  = canId;
    frame.can_dlc = dataLength;
    memcpy(frame.data, data, dataLength);

    if (write(can_sock, &frame, sizeof(struct can_frame)) == -1) {
        perror("sendCANFrame: write failed");
        can_close();
        exit(1);
    }
}

int receiveCANFrame(FOC_motor_t *out)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(can_sock, &read_fds);

    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = CAN_RECV_TIMEOUT_MS * 1000;

    int ret = select(can_sock + 1, &read_fds, NULL, NULL, &timeout);
    if (ret == 0) {
        printf("receiveCANFrame: timeout after %d ms\n", CAN_RECV_TIMEOUT_MS);
        return -1;
    }
    if (ret < 0) {
        perror("receiveCANFrame: select");
        return -2;
    }

    struct can_frame frame;
    ssize_t nbytes = read(can_sock, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("receiveCANFrame: read");
        return -2;
    }

    if (out) {
        out->position = uint16_to_float((frame.data[0] << 8 | frame.data[1]),
                                        -ONE_REV * GR, ONE_REV * GR);
        out->velocity = uint16_to_float((frame.data[2] << 8 | frame.data[3]),
                                        -SPEED_MAX,    SPEED_MAX);
        out->vq       = uint16_to_float((frame.data[4] << 8 | frame.data[5]),
                                        -V_MAX,        V_MAX);
        out->iq       = uint16_to_float((frame.data[6] << 8 | frame.data[7]),
                                        -I_MAX,        I_MAX);
        out->torque   = out->iq * KT;
    }
    return 0;
}
