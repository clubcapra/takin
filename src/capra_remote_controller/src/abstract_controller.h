#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class abstract_controller {

    public :
        /*geometry_msgs::Twist*/ void conversion(const sensor_msgs::Joy::ConstPtr& joy){};
        abstract_controller()=default;
};