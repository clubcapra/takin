#pragma once
#include <ros/ros.h>
#include "abstract_controller.h"

class controller_logitech : public abstract_controller{
    public : 
    /*geometry_msgs::Twist*/
    void conversion(const sensor_msgs::Joy::ConstPtr& joy);    
    controller_logitech();                        
};