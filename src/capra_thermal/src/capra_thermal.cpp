#include "NodeWrapper.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_thermal");

    ros::NodeHandle n;

    typedef sensor_msgs::Image Message;

    capra::NodeWrapper<Message> capra_thermal;

    // "IR_data" is a topic that the underlying driver publishes to.
    // subscribe_republish will subscribe to "IR_data".
    // subscribe_republish will republish to "capra_thermal/IR_data".
    capra_thermal.subscribe_republish(n, "IR_data", "capra_thermal/IR_data");

    /*
     * Testing NodeWrapper object functionalities with beginner_tutorials package.
     * The beginner_tutorials node publishes to "chatter" when run.
     */
    // capra::NodeWrapper<std_msgs::String> capra_chatter;
    // capra_chatter.subscribe_republish(n, "chatter", "capra_chatter", 
    //         [](const std_msgs::String::ConstPtr& msg) 
    //         {
    //             ROS_INFO("Lambda passed successfully as argument!");    
    //         });

    // int i = 5;
    // capra_chatter.add_observer([i](const std_msgs::String::ConstPtr& msg)
    //         {
    //             ROS_INFO("Captured variable is : %d", i);
    //         });

    ros::spin();

    return 0;
}