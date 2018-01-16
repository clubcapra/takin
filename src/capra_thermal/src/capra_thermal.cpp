#include "ThermalNode.h"
#include "sensor_msgs/Image.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_thermal");

    ros::NodeHandle n;

    typedef sensor_msgs::Image Message;
    typedef sensor_msgs::Image::ConstPtr MessageHandle;

    capra::NodeWrapper<Message, MessageHandle> capra_thermal;

    capra_thermal.subscribe_republish(n, "IR_data", "capra_thermal/IR_data");

    ros::spin();

    return 0;
}
