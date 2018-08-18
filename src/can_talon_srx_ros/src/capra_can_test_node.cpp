#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include "wpilib/CanTalonSRX.h"

#include "can_talon_srx/can_base.h"
#include "can_talon_srx/cansocket.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_talon_srx_node");
  auto nh = ros::NodeHandle();

  ROS_INFO("setting up CAN interface...");
  can_talon_srx::CanSocketInterface::Init("can0");
  ROS_INFO("CAN interface setup succesful!");

  CanTalonSRX testTalon1(11);
  CanTalonSRX testTalon2(12);
  CanTalonSRX testTalon3(61);
  CanTalonSRX testTalon4(62);

  std::atomic<bool> running(true);
  auto thr = std::thread([&]() {
    testTalon1.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon2.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon3.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    testTalon4.SetModeSelect(CanTalonSRX::kMode_DutyCycle, 100);
    int count = 0;
    while (running)
    {

      ROS_INFO("count %d",count);

      if ((count % 10) == 0)
      {
        int vals[4];
        testTalon1.GetEncPosition(vals[0]);
        testTalon2.GetEncPosition(vals[1]);
        testTalon3.GetEncPosition(vals[2]);
        testTalon4.GetEncPosition(vals[3]);
        ROS_INFO("position: %d %d %d %d %d %d %d %d",
            vals[0], vals[1], vals[2], vals[3]);
      }
      if ((count % 200) == 100)
      {
        testTalon1.SetDemand(200);
        testTalon2.SetDemand(200);
        testTalon3.SetDemand(200);
        testTalon4.SetDemand(200);
      }
      else if ((count % 200) == 199)
      {
        testTalon1.SetDemand(-200);
        testTalon2.SetDemand(-200);
        testTalon3.SetDemand(-200);
        testTalon4.SetDemand(-200);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ++count;
    }
  });

  ros::Rate update_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    update_rate.sleep();
  }
  running = false;
  thr.join();

  return 0;
}


