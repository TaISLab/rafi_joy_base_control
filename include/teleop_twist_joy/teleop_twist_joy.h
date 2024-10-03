/*

ROS Package: joy_base_control
File: teleop_twist_joy.h
Author: Rodrigo Castro Ochoa, rcastro@uma.es, University of Málaga.

Description:
Teleoperation of a omnidirectional mobile plattform.
This node subscribes to the topic /Joy and publishes a Twist message in the topic /cmd_vel.

Credits:
This code is based on the teleop_twist_joy project, available at: http://wiki.ros.org/teleop_twist_joy

*/

#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H

namespace ros { class NodeHandle; }

namespace teleop_twist_joy
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class TeleopTwistJoy
{
public:
  TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace teleop_twist_joy

#endif  // TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
