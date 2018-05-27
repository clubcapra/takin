#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "controller_logitech.h"


    /*geometry_msgs::Twist*/
    void controller_logitech::conversion(const sensor_msgs::Joy::ConstPtr& joy) {
        
        printf("%s\n", joy->axes[3]);
    }

    controller_logitech::controller_logitech(){};
    
