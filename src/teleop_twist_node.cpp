/*

ROS Package: joy_base_control
File: teleop_twist_node.cpp
Author: Rodrigo Castro Ochoa, rcastro@uma.es, University of Málaga.

Description:
Teleoperation of a omnidirectional mobile plattform.
This node subscribes to the topic /Joy and publishes a Twist message in the topic /cmd_vel.

Credits:
This code is based on the teleop_twist_joy project, available at: http://wiki.ros.org/teleop_twist_joy

*/

#include "ros/ros.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_twist_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
