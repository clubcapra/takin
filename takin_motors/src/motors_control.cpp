#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ros/ros.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
#include "sensor_msgs/Temperature.h"
#include "takin_msgs/BrakeMode.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>
#include <map>

std::vector<std::shared_ptr<TalonSRX>> left_track;
std::vector<std::shared_ptr<TalonSRX>> right_track;

ros::Subscriber cmd_vel_sub;

//front left, front right, rear left, rear right
std::vector<std::string> motor_names = {"drive_FL", "drive_FR", "drive_RL", "drive_RR"};


double clamp(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

void publishTemperature(ros::Publisher &temperaturePublisher) {
    int count = 0;
    for (auto &motor:left_track) {
        sensor_msgs::Temperature temperature;
        temperature.header.frame_id = motor_names[count++];
        temperature.header.stamp = ros::Time::now();
        temperature.temperature = motor->GetTemperature();
        temperaturePublisher.publish(temperature);
    }
    for (auto &motor:right_track) {
        sensor_msgs::Temperature temperature;
        temperature.header.frame_id = motor_names[count++];
        temperature.header.stamp = ros::Time::now();
        temperature.temperature = motor->GetTemperature();
        temperaturePublisher.publish(temperature);
    }
}

bool changeBrakeMode(takin_msgs::BrakeModeRequest &req, takin_msgs::BrakeModeResponse &res) {
    ctre::phoenix::motorcontrol::NeutralMode brakeMode = static_cast<ctre::phoenix::motorcontrol::NeutralMode>(req.brake_mode);

    for (auto &motor:left_track) {
        motor->SetNeutralMode(brakeMode);
    }
    for (auto &motor:right_track) {
        motor->SetNeutralMode(brakeMode);
    }
    return true;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    double linear = clamp(msg->linear.x, -1, 1);
    double angle = clamp(msg->angular.z, -1, 1);
    double power = clamp(std::sqrt(linear * linear + angle * angle), 0, 1);

    double left_power = angle > 0 ? power - 2 * angle : power;
    double right_power = angle < 0 ? power + 2 * angle : power;

    if (linear < 0) {
        left_power = -left_power;
        right_power = -right_power;
    }

    if (cmd_vel_sub.getNumPublishers() < 1) {
        ROS_ERROR("Detected multiple publishers. Only 1 publisher is allowed. Setting power to 0.");
        left_power = 0;
        right_power = 0;
    }

    ctre::phoenix::unmanaged::FeedEnable(100);

    for (auto &motor:left_track) {
        motor->Set(ControlMode::PercentOutput, left_power);
    }
    for (auto &motor:right_track) {
        motor->Set(ControlMode::PercentOutput, right_power);
    }
}

void setUpMotors(ros::NodeHandle &nh) {

    int FL, RL, FR, RR;
    if (nh.getParam("/motors_control/front_left", FL)) {
        left_track.push_back(std::make_shared<TalonSRX>(FL));
    }
    if (nh.getParam("/motors_control/rear_left", RL)) {
        left_track.push_back(std::make_shared<TalonSRX>(RL));
    }
    if (nh.getParam("/motors_control/front_right", FR)) {
        right_track.push_back(std::make_shared<TalonSRX>(FR));
    }
    if (nh.getParam("/motors_control/rear_right", RR)) {
        right_track.push_back(std::make_shared<TalonSRX>(RR));
    }

    // BUG : For some reason the drive 61 seems to be inverted. We need to set it back to normal
    left_track[1]->SetInverted(true);

    // BUG : The Follow function seems to cause a SEGFAULT
    // Assuming will always have an equal number of motors in both tracks
   /* for (int i = 1; i < left_track.size(); ++i) {
        //left_track[i]->Follow(*left_track[0].get());
    }

    for (int j = 1; j < right_track.size(); ++j) {
        //right_track[j]->Follow(*right_track[0].get());
    }*/

    ROS_INFO("Found motors, left: %d right: %d", (int)left_track.size(), (int)right_track.size());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "takin_motors_control");
    ros::NodeHandle nh;

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
    setUpMotors(nh);

    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, velocityCallback);
    ros::ServiceServer brake_mode_srv = nh.advertiseService("change_brake_mode", changeBrakeMode);
    ros::Publisher temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 1);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        publishTemperature(temperature_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



