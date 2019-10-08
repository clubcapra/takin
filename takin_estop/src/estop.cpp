#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"
#include "jetsonGPIO/jetsonGPIO.c"

bool estop_value = true; //default value for the Estop.
const jetsonTX2GPIONumber ESTOP_PIN = jetsonTX2GPIONumber::gpio389; //GPIO pin for the pin 21 on the J1 GPIO header
const jetsonTX2GPIONumber FEEDBACK_PIN = jetsonTX2GPIONumber::gpio389; //GPIO pin for the pin ## on the J1 GPIO header
int hw_version;

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopEnable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.message = "successfully toggle estop to on";
    gpioSetValue(ESTOP_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopDisable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.message = "successfully toggle estop to off";
    gpioSetValue(ESTOP_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    return true;
}

void publishEstopStatus(ros::NodeHandle *nh, ros::Publisher *pub){
    unsigned int status;
    jetsonTX2GPIONumber pin = nh->getParamCached("hw_version", hw_version) == 2 ? FEEDBACK_PIN : ESTOP_PIN;

    if (!gpioGetValue(pin, &status))
    {
        std_msgs::Bool msg;
        msg.data = (bool) status;
        pub->publish(msg);
    }
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
    ros::Publisher estop_status_pub = nh.advertise<std_msgs::Bool>("takin_estop_status", 1, true);
    initializeGPIO();

    ros::ServiceServer serviceEnable = nh.advertiseService("takin_estop_enable", toggleEstopEnable);
    ros::ServiceServer serviceDisable = nh.advertiseService("takin_estop_disable", toggleEstopDisable);

    ros::Rate r(60); // 60Hz
    while (ros::ok())
    {
        publishEstopStatus(&nh, &estop_status_pub);
        r.sleep(); // Might not be required, to be tested.
        ros::spinOnce();
        
    }

    return 0;
}


