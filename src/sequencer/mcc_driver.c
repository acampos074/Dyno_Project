#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "mcc_driver.h"
#include "pmd.h"
#include "usb-1608FS.h"

static libusb_device_handle *udev            = NULL;
static Calibration_AIN       table_AIN[NGAINS_USB1608FS][NCHAN_USB1608FS];

static const uint8_t MCC_CHANNEL       = 0;
static const uint8_t MCC_GAIN          = BP_5_00V;
static const float   NEWTONS_PER_VOLTS = 0.57f;

int mcc_init(void)
{
    int ret = libusb_init(NULL);
    if (ret < 0) {
        perror("mcc_init: libusb_init failed");
        return -1;
    }

    udev = usb_device_find_USB_MCC(USB1608FS_PID, NULL);
    if (!udev) {
        printf("mcc_init: USB-1608FS not found\n");
        return -1;
    }
    printf("mcc_init: USB-1608FS found\n");

    printf("mcc_init: building calibration table...\n");
    usbBuildCalTable_USB1608FS(udev, table_AIN);
    printf("mcc_init: calibration table ready\n");
    return 0;
}

float mcc_read_torque(void)
{
    signed short svalue = usbAIn_USB1608FS(udev, MCC_CHANNEL, MCC_GAIN, table_AIN);
    float volts = volts_USB1608FS(MCC_GAIN, svalue);
    return volts * NEWTONS_PER_VOLTS;
}

void mcc_close(void)
{
    if (udev) {
        libusb_close(udev);
        udev = NULL;
    }
    libusb_exit(NULL);
}
