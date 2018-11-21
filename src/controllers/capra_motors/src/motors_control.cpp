#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <capra_motors/MotorConfig.h>

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
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

std::vector<std::unique_ptr<TalonSRX>> left_track;
std::vector<std::unique_ptr<TalonSRX>> right_track;

double clamp(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

void configCallback(capra_motors::MotorConfig &config, uint32_t level) {

    ctre::phoenix::motorcontrol::NeutralMode brakeMode;
    switch (config.brake_mode) {
        case 1: brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Coast; break;
        default: brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Brake; break;
    }

    for (auto &motor:left_track) {
        motor->SetNeutralMode(brakeMode);
    }
    for (auto &motor:right_track) {
        motor->SetNeutralMode(brakeMode);
    }
}

void velocityCallback(const geometry_msgs::Twist &msg) {

    double linear = clamp(msg.linear.x, -1, 1);
    double angle = clamp(msg.angular.z, -1, 1);
    double power = std::sqrt(linear * linear + angle * angle);

    double left_power = angle > 0 ? (1.0 - 2 * angle) * power : power;
    double right_power = angle < 0 ? (1.0 + 2 * angle) * power : power;

    if(linear < 0)
    {
        left_power = -left_power;
        right_power = -right_power;
    }

    ctre::phoenix::unmanaged::FeedEnable(100);

    for (auto &motor:left_track) {
        motor->Set(ControlMode::PercentOutput, left_power);
    }
    for (auto &motor:right_track) {
        motor->Set(ControlMode::PercentOutput, right_power);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "capra_motors_control");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<capra_motors::MotorConfig> server;
    dynamic_reconfigure::Server<capra_motors::MotorConfig>::CallbackType f = boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    left_track.push_back(std::make_unique<TalonSRX>(RL));
    right_track.push_back(std::make_unique<TalonSRX>(RR));

    for (auto &motor:left_track) {
        motor->SetInverted(true);
    }

    ros::Subscriber remote_control = n.subscribe("cmd_vel", 1000, velocityCallback);
    ros::spin();
    return 0;
}



