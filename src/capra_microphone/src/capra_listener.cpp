#include "ros/ros.h"
#include "std_msgs/String.h"
#include "audio_common_msgs/AudioData.h"
#include "iostream"
#


void listen(const audio_common_msgs::AudioDataConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", "test");
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "capra_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/audio/audio", 10, listen);

    ros::spin();
 
    return 0;
}
