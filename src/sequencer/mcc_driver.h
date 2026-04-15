#ifndef MCC_DRIVER_H
#define MCC_DRIVER_H

/*
 * Initialize the MCC USB-1608FS DAQ device.
 * Finds the device, builds the calibration table.
 * Returns 0 on success, -1 if the device was not found or init failed.
 */
int   mcc_init(void);

/*
 * Read one torque sample from analog channel 0 (±5 V range).
 * Converts ADC counts → volts → Nm using 0.57 Nm/V calibration.
 * Must call mcc_init() before first use.
 */
float mcc_read_torque(void);

/* Release the USB device handle and call libusb_exit(). */
void  mcc_close(void);

#endif /* MCC_DRIVER_H */
