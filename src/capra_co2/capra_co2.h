
//#include <glib/gprintf.h>
//#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define ADDR_6713  0x15 // default I2C slave address

//int GetCO2PPM(int addr){};
