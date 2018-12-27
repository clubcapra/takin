#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ros/ros.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <dynamic_reconfigure/server.h>
//#include <takin_motors/MotorConfig.h>
#include "sensor_msgs/Temperature.h"



#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <memory>

std::vector<std::shared_ptr<TalonSRX>> left_track;
std::vector<std::shared_ptr<TalonSRX>> right_track;

ros::Subscriber remote_controls_subscriber;

double clamp(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
}

void publishingMotorsTemperature(std::vector<ros::Publisher> &motorTemperaturePublishers,
                                 std::vector<std::shared_ptr<TalonSRX>> &motorTrack) {
    for (int i = 0; i < motorTrack.size(); i++) {
        sensor_msgs::Temperature tmpTemperature;
        tmpTemperature.temperature = motorTrack.at(i)->GetTemperature();
        motorTemperaturePublishers[i].publish(tmpTemperature);
    }
}



//void configCallback(takin_motors::MotorConfig &config, uint32_t level) {
//
//    ctre::phoenix::motorcontrol::NeutralMode brakeMode;
//    switch (config.brake_mode) {
//        case 1:
//            brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
//            break;
//        default:
//            brakeMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
//            break;
//    }
//
//    for (auto &motor:left_track) {
//        motor->SetNeutralMode(brakeMode);
//    }
//    for (auto &motor:right_track) {
//        motor->SetNeutralMode(brakeMode);
//    }
//}

//void velocityCallback(const geometry_msgs::Twist &msg, boost::_bi::value<ros::Subscriber> &remote_control) {
void velocityCallback(const geometry_msgs::Twist::ConstPtr &msg) {

    double linear = clamp(msg->linear.x, -1, 1);
    double angle = clamp(msg->angular.z, -1, 1);
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

    if (remote_controls_subscriber.getNumPublishers() < 1) {
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
    //front left, front right, rear left, rear right
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
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "takin_motors_control");
    ros::NodeHandle nh;

//    dynamic_reconfigure::Server<takin_motors::MotorConfig> server;
//    dynamic_reconfigure::Server<takin_motors::MotorConfig>::CallbackType f = boost::bind(&configCallback, _1, _2);
//    server.setCallback(f);

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());
    setUpMotors(nh);

    remote_controls_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, velocityCallback);
    // ROS pub motors temp
    ros::Publisher temperaturePublisherFL = nh.advertise<sensor_msgs::Temperature>("temperature_FL", 1000);
    ros::Publisher temperaturePublisherFR = nh.advertise<sensor_msgs::Temperature>("temperature_FR", 1000);
    ros::Publisher temperaturePublisherRL = nh.advertise<sensor_msgs::Temperature>("temperature_RL", 1000);
    ros::Publisher temperaturePublisherRR = nh.advertise<sensor_msgs::Temperature>("temperature_RR", 1000);

    std::vector<ros::Publisher> leftTrackTemperaturePublishers = {temperaturePublisherFL, temperaturePublisherRL};
    std::vector<ros::Publisher> rightTrackTemperaturePublishers = {temperaturePublisherFR, temperaturePublisherRR};

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        publishingMotorsTemperature(leftTrackTemperaturePublishers, left_track);
        publishingMotorsTemperature(rightTrackTemperaturePublishers, right_track);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



