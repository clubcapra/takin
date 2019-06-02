#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "jetsonGPIO/jetsonGPIO.c"

bool estop_value = true; //default value for the Estop.
const jetsonTX2GPIONumber ESTOP_PIN = jetsonTX2GPIONumber::gpio298; //GPIO pin for the pin 21 on the J1 GPIO header

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    estop_value = !estop_value;

    if (estop_value) {
        res.message = "successfully toggle estop to on";
        gpioSetValue(ESTOP_PIN, 1);
    } else {
        res.message = "successfully toggle estop to off";
        gpioSetValue(ESTOP_PIN, 0);
    }
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO() {
    gpioExport(ESTOP_PIN);
    gpioSetDirection(ESTOP_PIN, 1);
    gpioSetValue(ESTOP_PIN, 1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "takin_estop");
    ros::NodeHandle nh;

    initializeGPIO();

    ros::ServiceServer service = nh.advertiseService("takin_estop", toggleEstop);
    ros::spin();

    return 0;
}


