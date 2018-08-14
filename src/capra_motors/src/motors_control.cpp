#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


class CapraMotorControl
{
public:
  CapraMotorControl();
  static int FR_ID, FL_ID, RR_ID, RL_ID;

private:
  void move(int motorId, bool direction);
  
  ros::NodeHandle nh_;


};

static int FL_ID = 11;
static int FR_ID = 12;
static int RL_ID = 61;
static int RR_ID = 62;




/*
*/

CapraMotorControl::CapraMotorControl()
{
}


/*
Callback function so that everytime joy publish the node catch it and convert 
it.
*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "capra_motors_control");
  CapraMotorControl capra_motors_control;

  //create the six motors

  ros::spin();
 
}
