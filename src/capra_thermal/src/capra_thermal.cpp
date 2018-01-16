#include "ThermalNode.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_thermal");

    ros::NodeHandle n;

    ThermalNode arrayNode;

    arrayNode._array_pub = n.advertise<std_msgs::Int32MultiArray>("capra_thermal/array", 50);
    arrayNode._IR_pub = n.advertise<sensor_msgs::Image>("capra_thermal/IR_data", 50);
    arrayNode._array_sub = n.subscribe("array", 50, &ThermalNode::repost_array, &arrayNode);
    arrayNode._IR_sub = n.subscribe("IR_data", 50, &ThermalNode::repost_IR, &arrayNode);

    ros::spin();

    return 0;
}