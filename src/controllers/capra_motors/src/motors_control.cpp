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

void changeBrakeMode(const sensor_msgs::Joy::ConstPtr &joy);

void addLeftMotor(std::vector<TalonSRX *> &left_track, TalonSRX *motor);

int main(int argc, char **argv);

int main(int argc, char **argv) {
    ros::init(argc, argv, "capra_motors_control");
    ros::NodeHandle n;
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    talonRL = new TalonSRX(RL);
    talonRR = new TalonSRX(RR);

/*   addLeftMotor(left_track, new TalonSRX(FL));
    right_track.push_back(new TalonSRX(FR));*/
    addLeftMotor(left_track, talonRL);
    right_track.push_back(talonRR);

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

    //brakeMotors(joy);

    moveMotors(joy);

    changeBrakeMode(joy);
}


void changeBrakeMode(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        if (joy->buttons[4] == 1 && !pressedBrakeMode) {
            pressedBrakeMode = true;
        } else if (joy->buttons[4] == 0 && pressedBrakeMode) {
            brakeMode = !brakeMode;
            pressedBrakeMode = false;
        }
        if (brakeMode) {
            for (auto &motor:both_tracks) {
                motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
            }
        } else {
            for (auto &motor:both_tracks) {
                motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
            }
        }
    }
}


void checkFeedEnable(const sensor_msgs::Joy::ConstPtr &joy) {
    if (!pressed && joy->buttons[0] == 1 && joy->buttons[6] == 1) {
        pressed = true;
    } else if (joy->buttons[0] == 0 && joy->buttons[6] == 0 && pressed) {
        feedEnableToggle = !feedEnableToggle;
        pressed = false;
    }
    ROS_INFO("FEED ENABLE : %d", feedEnableToggle);
}


void brakeMotors(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        if (joy->axes[2] != 1.0) {
            ctre::phoenix::unmanaged::FeedEnable(100);
            for (auto &motor:both_tracks) {
                motor->Set(ControlMode::PercentOutput, 0.0);
            }
        }
    }
}

void moveMotors(const sensor_msgs::Joy::ConstPtr &joy) {
    if (feedEnableToggle) {
        double power = 1 - (joy->axes[5] + 1) / 2;
        double x_axe = joy->axes[0];
        double y_axe = joy->axes[1];
        double left_power;
        double right_power;
        ctre::phoenix::unmanaged::FeedEnable(100);

        // Brake function
        if (joy->axes[2] != 1.0) {
            left_power = 0.0;
            right_power = 0.0;
        }
            // Move L=1 and R=1
        else if (x_axe < 0.25 && x_axe > -0.25 && y_axe > 0.25 && power > 0) {
            left_power = power;
            right_power = power;
        }
            // Move L=1 and R=0
        else if (x_axe < -0.25 && x_axe > -0.75 && y_axe > 0.25 && power > 0) {
            left_power = power;
            right_power = 0.0;
        }
            // Move L=1 and R=-1
        else if (x_axe < -0.25 && y_axe < 0.25 && y_axe > -0.25 && power > 0) {
            left_power = power;
            right_power = power * -1;
        }
            // Move L=-1 and R=0
        else if (x_axe < -0.25 && y_axe < -0.25 && y_axe > -0.75 && power > 0) {
            left_power = power * -1;
            right_power = 0.0;
        }
            // Move L=-1 and R=-1
        else if (x_axe < 0.25 && x_axe > -0.25 && y_axe < -0.25 && power > 0) {
            left_power = power * -1;
            right_power = power * -1;
        }
            // Move L=0 and R=-1
        else if (x_axe > 0.25 && x_axe < 0.75 && y_axe < -0.25 && power > 0) {
            left_power = 0.0;
            right_power = power * -1;
        }
            // Move L=-1 and R=1
        else if (x_axe > 0.25 && y_axe < -0.25 && y_axe < 0.25 && power > 0) {
            left_power = power * -1;
            right_power = power;
        }
            // Move L=0 and R=1
        else if (x_axe > 0.25 && y_axe > 0.25 && y_axe < 0.75 && power > 0) {
            left_power = 0.0;
            right_power = power;
        }
            // Default if not direction chosen
        else {
            left_power = 0.0;
            right_power = 0.0;
        }

        for (auto &motor:left_track) {
            motor->Set(ControlMode::PercentOutput, left_power);
        }
        for (auto &motor:right_track) {
            motor->Set(ControlMode::PercentOutput, right_power);
        }
    }
}




