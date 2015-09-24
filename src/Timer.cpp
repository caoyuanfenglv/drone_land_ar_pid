#include "ros/ros.h"

/**
 * This tutorial demonstrates the use of timer callbacks.
 */

void callback1(const ros::TimerEvent&)
{
  ROS_INFO("Callback 1 triggered");
}

void callback2(const ros::TimerEvent& e)
{
  ROS_INFO("Callback 2 triggered");
  std::cout<<e.current_real<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  /**
   * Timers allow you to get a callback at a specified rate.  Here we create
   * two timers at different rates as a demonstration.
   */
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);

  ros::spin();

  return 0;
}