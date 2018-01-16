#include "ThermalNode.h"

void ThermalNode::repost_array(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    this->_array_pub.publish(*msg);
}

void ThermalNode::repost_IR(const sensor_msgs::Image::ConstPtr& msg)
{
    this->_IR_pub.publish(*msg);
}
