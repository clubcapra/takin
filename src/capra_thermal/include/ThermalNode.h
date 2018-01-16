#ifndef THERMAL_NODE_H
#define THERMAL_NODE_H

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Image.h"

class ThermalNode {
    public:
        void repost_array(const std_msgs::Int32MultiArray::ConstPtr& msg);
        void repost_IR(const sensor_msgs::Image::ConstPtr& msg);

        ros::Publisher _array_pub;
        ros::Publisher _IR_pub;
        ros::Subscriber _array_sub;
        ros::Subscriber _IR_sub;
};

#endif // THERMAL_NODE_H