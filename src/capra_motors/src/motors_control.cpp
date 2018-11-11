#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "Platform-linux-socket-can.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "MotorPhoenix.h"
#include <sensor_msgs/Joy.h>
//

// SDL code from https://gist.github.com/fabiocolacio/423169234b8daf876d8eb75d8a5f2e95

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

bool feedEnableToggle = false;
bool pressed;

MotorPhoenix motor_FL(11, MotorPhoenix::LEFT_MOTOR);
MotorPhoenix motor_FR(12, MotorPhoenix::RIGHT_MOTOR);
MotorPhoenix motor_RL(61, MotorPhoenix::LEFT_MOTOR);
MotorPhoenix motor_RR(62, MotorPhoenix::RIGHT_MOTOR);

void joystickCallback(const sensor_msgs::Joy::ConstPtr &joy) {

    if (!pressed && joy->buttons[0] == 1 && joy->buttons[6] == 1) {
        pressed = true;
    } else if (joy->buttons[0] == 0 && joy->buttons[6] == 0 && pressed) {
        feedEnableToggle = !feedEnableToggle;
        pressed = false;
    }

    if (feedEnableToggle) {
        if (joy->axes[2] != 1.0) {
            ctre::phoenix::unmanaged::FeedEnable(100);
            motor_FL.setPercentOutput(0.0);
        } else {
            ctre::phoenix::unmanaged::FeedEnable(100);
            motor_FL.setPercentOutput(joy->axes[1]);
        }
    }
}

int main(int argc, char **argv);

int main(int argc, char **argv) {
    ros::init(argc, argv, "capra_motors_control");
    ros::NodeHandle n;
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
    ros::Subscriber remote_control = n.subscribe("capra_rc_joy", 1000, joystickCallback);
    ros::spin();
    return 0;
}
