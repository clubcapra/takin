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

#define RR 62
#define RL 61
#define FR 12
#define FL 11

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

bool feedEnableToggle = false;
bool brakeMode; // True = coast False = brake
bool pressed;
bool pressedBrakeMode;


TalonSRX *talonFL;
TalonSRX *talonFR;
TalonSRX *talonRL;
TalonSRX *talonRR;

std::vector<TalonSRX *> left_track;
std::vector<TalonSRX *> right_track;
std::vector<TalonSRX *> both_tracks;

void joystickCallback(const sensor_msgs::Joy::ConstPtr &joy);

void checkFeedEnable(const sensor_msgs::Joy::ConstPtr &joy);

void moveMotors(const sensor_msgs::Joy::ConstPtr &joy);

void brakeMotors(const sensor_msgs::Joy::ConstPtr &joy);

void checkBrakeMode(const sensor_msgs::Joy::ConstPtr &joy);

void changeBrakeMode(bool brakeMode);

void addLeftMotor(std::vector<TalonSRX *> &left_track, TalonSRX *motor);

int main(int argc, char **argv);

int main(int argc, char **argv) {
    ros::init(argc, argv, "capra_motors_control");
    ros::NodeHandle n;
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

/*   addLeftMotor(left_track, new TalonSRX(FL));
    right_track.push_back(new TalonSRX(FR));*/
    addLeftMotor(left_track, new TalonSRX(RL));
    right_track.push_back(new TalonSRX(RR));

    both_tracks.reserve(left_track.size() + right_track.size());
    both_tracks.insert(both_tracks.end(), left_track.begin(), left_track.end());
    both_tracks.insert(both_tracks.end(), right_track.begin(), right_track.end());


    ros::Subscriber remote_control = n.subscribe("capra_rc_joy", 1000, joystickCallback);
    ros::spin();
    return 0;
}

void addLeftMotor(std::vector<TalonSRX *> &left_track, TalonSRX *motor) {
    motor->SetInverted(true);
    left_track.push_back(motor);
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr &joy) {

    checkFeedEnable(joy);

    brakeMotors(joy);

    moveMotors(joy);

    checkBrakeMode(joy);

    changeBrakeMode(brakeMode);
}


void changeBrakeMode(bool brakeMode) {
    if (brakeMode) {
        for (auto &motor:both_tracks) {
            motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
            ROS_INFO("Setting brake mode to Coast");
        }
    } else {
        for (auto &motor:both_tracks) {
            motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
            ROS_INFO("Setting brake mode to Brake");
        }
    }
}

void checkBrakeMode(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        if (joy->buttons[4] == 1 && !pressedBrakeMode) {
            pressedBrakeMode = true;
        } else if (joy->buttons[4] == 0 && pressedBrakeMode) {
            brakeMode = !brakeMode;
            pressedBrakeMode = false;
        }
    }
    ROS_INFO("BrakeMode value is : %d", brakeMode);
}


void checkFeedEnable(const sensor_msgs::Joy::ConstPtr &joy) {
    if (!pressed && joy->buttons[0] == 1 && joy->buttons[6] == 1) {
        pressed = true;
    } else if (joy->buttons[0] == 0 && joy->buttons[6] == 0 && pressed) {
        feedEnableToggle = !feedEnableToggle;
        pressed = false;
    }
}


void brakeMotors(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        if (joy->axes[2] != 1.0 && joy->axes[1] != 0.0) {
            ctre::phoenix::unmanaged::FeedEnable(100);
            for (auto &motor:both_tracks) {
                motor->Set(ControlMode::PercentOutput, 0.0);
            }
        }
    }
}

void moveMotors(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        // Move L=1 and R=1
        if (joy->axes[1] > 0.0 && (1 - (joy->axes[5] + 1) / 2) > 0) {
            ctre::phoenix::unmanaged::FeedEnable(100);
            ROS_INFO("MOTOR INPUT %f", (1 - (joy->axes[5] + 1) / 2));
            for (auto &motor:both_tracks)
                motor->Set(ControlMode::PercentOutput, 1 - (joy->axes[5] + 1) / 2);
        }
            // Move L=1 and R=0
        else if (joy->axes[1] > 0.0 && (1 - (joy->axes[5] + 1) / 2) > 0) {}
            // Move L=1 and R=-1
        else if () {}
            // Move L=0 and R=-1
        else if () {}
            // Move L=-1 and R=-1
        else if () {}
            // Move L=-1 and R=0
        else if () {}
            // Move L=-1 and R=1
        else if () {}
            // Move L=0 and R=1
        else if () {}

    }
}





