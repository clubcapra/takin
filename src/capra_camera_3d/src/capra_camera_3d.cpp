#include "NodeWrapper.h"
// #include "PACKAGE/MSG TYPE HEADER"

#include <chrono>
#include <thread>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "capra_camera_3d");

    ros::NodeHandle n;

    // Make this process sleep for 5 seconds
    // to let 3D camera initialize all its
    // topics
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10s);

    /*
    typedef <MSG TYPE HERE> Message;

    capra::NodeWrapper<Message> capra_camera_3d;

    capra_camera_3d.subscribe_republish(n, <"TOPIC NAME">, <"capra_camera_3d/TOPIC NAME">);
     */

    ros::spin();
    return 0;
}