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
#include <sensor_msgs/Joy.h>
#include <vector>
#include <memory>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

bool feedEnableToggle = false;
bool pressed;

std::vector<TalonSRX> left_track;
std::vector<TalonSRX> right_track;
std::vector<TalonSRX> both_tracks;

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
            for (auto &motor:both_tracks) {

                motor.Set(ControlMode::PercentOutput, 0.0);
            }
        } else if (joy->axes[1] > 0.0) { // Forward
            ctre::phoenix::unmanaged::FeedEnable(100);
            for (auto &motor:both_tracks)
                motor.Set(ControlMode::PercentOutput, 1.0 - joy->axes[5]);
        }
    }
}

int main(int argc, char **argv);

int main(int argc, char **argv) {
    ros::init(argc, argv, "capra_motors_control");
    ros::NodeHandle n;
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    left_track.emplace_back(11);
    left_track.emplace_back(61);
    right_track.emplace_back(12);
    right_track.emplace_back(62);

    both_tracks.reserve(left_track.size() + right_track.size());
    both_tracks.insert(both_tracks.end(), left_track.begin(), left_track.end());
    both_tracks.insert(both_tracks.end(), right_track.begin(), right_track.end());


    ros::Subscriber remote_control = n.subscribe("capra_rc_joy", 1000, joystickCallback);
    ros::spin();
    return 0;
}
