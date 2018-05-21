#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>




class CapraMotorCmdVel
{
public:
  CapraMotorCmdVel();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_x, linear_y;
  int angular_x, angular_y;

  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};



/*
Create a nodethat uses the axes 3 and 4 for the linear value and 6 and 7 for the angular value. 
It convert it  to a cmd_vel format. 
*/

CapraMotorCmdVel::CapraMotorCmdVel(): 
  linear_x(3),
  linear_y(4),
  angular_x(6),
  angular_y(7)
{

  nh_.param("axis_linear", linear_x, linear_x);
  nh_.param("axis_angular", linear_y, linear_y);
  nh_.param("axis_angular", angular_x, angular_x);
  nh_.param("axis_angular", angular_y, angular_y);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_angular", a_scale_, a_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("capra_motors/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &CapraMotorCmdVel::joyCallback, this);

}


/*
Callback function so that everytime joy publish the node catch it and convert 
it.
*/
void CapraMotorCmdVel::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.linear.y = l_scale_*joy->axes[linear_y];
  twist.linear.x = l_scale_*joy->axes[linear_x];
  twist.angular.y = a_scale_*joy->axes[angular_y];
  twist.angular.x = a_scale_*joy->axes[angular_x];
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "capra_motors_cmd_vel");
  CapraMotorCmdVel capra_motors_cmd_vel;

  ros::spin();
// 
}
