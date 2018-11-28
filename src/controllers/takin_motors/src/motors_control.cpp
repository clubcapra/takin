#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ros/ros.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include <takin_motors/MotorConfig.h>

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>

std::vector<std::shared_ptr<TalonSRX>> left_track;
std::vector<std::shared_ptr<TalonSRX>> right_track;

double clamp(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

void configCallback(takin_motors::MotorConfig &config, uint32_t level) {

    ctre::phoenix::motorcontrol::NeutralMode brakeMode;
    switch (config.brake_mode) {
        case 1:
            brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
            break;
        default:
            brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
            break;
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

    if (power != 0.0) {
        angle /= power;
        linear /= power;
    }

    double left_power = angle > 0 ? (1.0 - 2 * angle) * power : power;
    double right_power = angle < 0 ? (1.0 + 2 * angle) * power : power;

    if (linear < 0) {
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

    ros::init(argc, argv, "takin_motors_control");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<takin_motors::MotorConfig> server;
    dynamic_reconfigure::Server<takin_motors::MotorConfig>::CallbackType f = boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    int FL, FR, RL, RR;
    if (n.getParam("/motors_control/front_left", FL)) {
        ROS_INFO("front left motor detected : %d", FL);
        left_track.push_back(std::make_shared<TalonSRX>(FL));
    }
    if (n.getParam("/motors_control/front_right", FR)) {
        ROS_INFO("front right motor detected : %d", FR);
        right_track.push_back(std::make_shared<TalonSRX>(FR));
    }
    if (n.getParam("/motors_control/rear_left", RL)) {
        ROS_INFO("front left motor detected : %d", RL);
        left_track.push_back(std::make_shared<TalonSRX>(RL));
    }
    if (n.getParam("/motors_control/rear_right", RR)) {
        ROS_INFO("front left motor detected : %d", RR);
        right_track.push_back(std::make_shared<TalonSRX>(RR));
    }
    // Assuming will always have an equal number of motors in both tracks
    for (int i = 1; i < left_track.size(); ++i) {
        left_track[i]->Follow(*left_track[0].get());
    }

    for (int j = 1; j < right_track.size(); ++j) {
        right_track[j]->Follow(*right_track[0].get());
    }


    for (auto &motor:left_track) {
        motor->SetInverted(true);
    }
    ros::Subscriber remote_control = n.subscribe("cmd_vel", 1000, velocityCallback);
    ros::spin();
    return 0;
}



