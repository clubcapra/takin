// ROS framework tools
#include "ros/ros.h"
#include "ros/service_client.h"

// Message to publish
#include "nav_msgs/Odometry.h"

// Message to convert and forward
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/GetLinkStateRequest.h"
#include "gazebo_msgs/GetLinkStateResponse.h"

#define LINK_NAME (1)
#define REFERENCE_FRAME (2)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "capra_odometry_simulation_publisher");

    // Make this a node
    ros::NodeHandle node;

    ROS_INFO("%s", argv[LINK_NAME]);
    ROS_INFO("%s", argv[REFERENCE_FRAME]);

    // Declare Odometry publishing msg types
    ros::Publisher capra_odometry_publisher = 
        node.advertise<nav_msgs::Odometry>("capra/simulation/odom", 1000);

    // Create a persistent proxy from which to call gazebo's GetLinkState service
    ros::ServiceClient proxy = 
        node.serviceClient<gazebo_msgs::GetLinkState>(
            "/gazebo/get_link_state",
            true
        );

    // Get the odometry struct from which we publish odometry msgs
    nav_msgs::Odometry capra_odometry;

    // Set its frame_id to /odom for tf to use
    capra_odometry.header.frame_id = "/odom";
    capra_odometry.child_frame_id = "base_link";

    // Create the request
    gazebo_msgs::GetLinkStateRequest link_state_request;

    // Give the request necessary argument
    // link_state_request.link_name = "capra6::base_link";
    // link_state_request.reference_frame = "world";
    link_state_request.link_name = argv[LINK_NAME];
    link_state_request.reference_frame = argv[REFERENCE_FRAME];

    // Get a placeholder for the service response
    gazebo_msgs::GetLinkStateResponse link_state_response;

    // Set service call rate
    ros::Rate loop_rate(10);

    // Blocks until service is available
    proxy.waitForExistence();

    while (node.ok())
    {

        if(!proxy.isValid())
        {
            proxy = node.serviceClient<gazebo_msgs::GetLinkState>(
                "/gazebo/get_link_state",
                true);

            proxy.waitForExistence();
        }

        while (proxy.call(link_state_request, link_state_response))
        {
            // Build odometry message with the link state response
            capra_odometry.header.stamp = ros::Time::now();

            capra_odometry.pose.pose =
                link_state_response.link_state.pose;

            capra_odometry.twist.twist = 
                link_state_response.link_state.twist;

            capra_odometry_publisher.publish(capra_odometry);

            loop_rate.sleep();
        }
    }

    return EXIT_SUCCESS;
}