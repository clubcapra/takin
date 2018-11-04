#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <memory>

class CapraRemoteController
{
public:
  CapraRemoteController();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  ros::NodeHandle nh;

  ros::Publisher joy_pub;
  ros::Subscriber joy_sub;
};

/*
Create a node that uses the axes 3 and 4 for the linear value and 6 and 7 for the angular value. 
It convert it  to a cmd_vel format. 
*/

CapraRemoteController::CapraRemoteController()
{

  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 100, &CapraRemoteController::joyCallback, this);
  joy_pub = nh.advertise<sensor_msgs::Joy>("capra_rc_joy", 1);
}

/*
Callback function so that everytime joy publish the node catch it and convert 
it.
*/
void CapraRemoteController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  joy_pub.publish(joy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capra_remote_controller");
  CapraRemoteController capra_remote_controller;

  ros::spin();
}
