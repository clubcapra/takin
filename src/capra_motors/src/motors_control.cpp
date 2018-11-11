#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "Platform-linux-socket-can.h"
#include <SDL2/SDL.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>

// SDL code from https://gist.github.com/fabiocolacio/423169234b8daf876d8eb75d8a5f2e95

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

bool feedEnableToggle = false;
bool pressed;

void joystickCallback(const sensor_msgs::Joy::ConstPtr &joy) {

    TalonSRX *talonRL;
    talonRL = new TalonSRX(62);
    /*     TalonSRX *talonRL = new TalonSRX(61);
    TalonSRX *talonFL = new TalonSRX(12);
    TalonSRX *talonRL = new TalonSRX(62); 
    */

    if (!pressed && joy->buttons[0] == 1 && joy->buttons[6] == 1) {
        pressed = true;
    } else if (joy->buttons[0] == 0 && joy->buttons[6] == 0 && pressed) {
        feedEnableToggle = !feedEnableToggle;
    }

    ROS_INFO("FEEDENDABLE : %b", feedEnableToggle);
//    if (feedEnableToggle) {
//        ROS_INFO("TEST MOTOR CONTROL");
//        if (joy->axes[2] == 1.0) {
//            ROS_INFO("TEST MOTOR FORWARD");
//            ctre::phoenix::unmanaged::FeedEnable(100);
//            talonRL->Set(ControlMode::PercentOutput, joy->axes[2]);
//        } else {
//            ctre::phoenix::unmanaged::FeedEnable(100);
//            talonRL->Set(ControlMode::PercentOutput, 0.0);
//        }
//    }
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
