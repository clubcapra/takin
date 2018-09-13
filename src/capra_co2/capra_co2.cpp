#include "capra_co2.h"

int main(void) {
    int file;
    char filename[40];
    //char *buffer;
    int addr = ADDR_6713;        // The I2C address of the ADC

    sprintf(filename,"/dev/i2c-2");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    char buf[4] = {0};
    int ppmValue = 0;

    for(int i = 0; i<4; i++) {
        // Using I2C Read
        if (read(file,buf,2) != 2) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            //buffer = g_strerror(errno);
            //printf(buffer);
            //printf("\n\n");
        } else {
            ppmValue = (int)((buf[2] & 0x3F) << 8) | buf[3];
            printf("C02 ppm: %04d\n",ppmValue);
        }
    }

    //unsigned char reg = 0x10; // Device register to access
    //buf[0] = reg;
    //buf[0] = 0b11110000;

    if (write(file,buf,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        //buffer = g_strerror(errno);
        //printf(buffer);
        //printf("\n\n");
    }
    return EXIT_SUCCESS;
}
