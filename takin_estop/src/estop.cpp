#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "jetsonGPIO/jetsonGPIO.c"

bool estop_status = true;
jetsonTX2GPIONumber estop_pin = jetsonTX2GPIONumber::gpio388;

bool toggleEstop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    bool prev_estop = estop_status;
    estop_status = !estop_status;

    if (estop_status != prev_estop) {
        res.success = static_cast<unsigned char>(true);
        res.message = "successfully toggle estop";
        (estop_status) ? gpioSetValue(estop_pin,1) : gpioSetValue(estop_pin,0);
    } else {
        res.success = static_cast<unsigned char>(false);
        res.message = "failed toggle estop";
    }

    return true;
}

void initializeGPIO() {
    gpioExport(estop_pin);
    gpioSetDirection(estop_pin, 1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "takin_estop");
    ros::NodeHandle nh;

    initializeGPIO();

    ros::ServiceServer service = nh.advertiseService("takin_estop", toggleEstop);
    ros::spin();

    return 0;
}


