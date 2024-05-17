/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>

namespace teleop_twist_joy
{

  /**
   * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
   * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
   * directly into base nodes.
   */
  struct TeleopTwistJoy::Impl
  {
    // Members fuctions
    void printTwistInfo(const geometry_msgs::Twist &velocity, const std::string &info_string);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void ModifyVelocity(const sensor_msgs::Joy::ConstPtr &joy_msg, float &scale, float &max_vel);

    // ROS subscribers and publisher
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub;

    geometry_msgs::Twist cmd_vel_msg;

    int enable_mov; // Activa el movimiento
    int increment_vel;
    int decrement_vel;
    float mov_vel;
    float orientation_vel;
    float min_vel = 2;
    float max_vel;

    float reaction_t = 0.5; // Tiempo en segundo de reaccion del operador
    float Delta_t = 0.01;   // Tiempo en segundo

    std::map<std::string, int> axis_position_map;
    std::map<std::string, int> axis_orientation_map;
  };

  /**
   * Constructs TeleopTwistJoy.
   * \param nh NodeHandle to use for setting up the publisher and subscriber.
   * \param nh_param NodeHandle to use for searching for configuration parameters.
   */
  TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle *nh, ros::NodeHandle *nh_param)
  {
    pimpl_ = new Impl;

    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

    // Asignar botones
    nh_param->param<int>("enable_mov", pimpl_->enable_mov, 0);
    nh_param->param<int>("increment_velocity", pimpl_->increment_vel, -1);
    nh_param->param<int>("decrement_velocity", pimpl_->decrement_vel, -1);

    // Asignación de mapas
    nh_param->getParam("axis_position_map", pimpl_->axis_position_map);
    nh_param->getParam("axis_orientation_map", pimpl_->axis_orientation_map);

    nh_param->getParam("max_displacement_in_a_second", pimpl_->max_vel);
  }

  void TeleopTwistJoy::Impl::printTwistInfo(const geometry_msgs::Twist &velocity, const std::string &info_string)
  {
    ROS_INFO("%s - Linear (x, y, z): (%.5f, %.5f, %.5f), Angular (x, y, z, w): (%.5f, %.5f, %.5f)",
             info_string.c_str(),
             velocity.linear.x, velocity.linear.y, velocity.linear.z,
             velocity.angular.x, velocity.angular.y, velocity.angular.z);
  }

  double getVal(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_map, const std::string &fieldname)
  {
    /*
    Método que obtiene valores especificos del mensaje del joystick:
    Argumentos:
      - joy_msg: mensaje joy del cual se va a obtener la informacion
      - axis_map: mapa de ejes de control
      - fieldname: campo que se quiere obtener [x,y,z] o [x,y,z,w]
    */

    if (axis_map.find(fieldname) == axis_map.end() || joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)];
  }

  void TeleopTwistJoy::Impl::ModifyVelocity(const sensor_msgs::Joy::ConstPtr &joy_msg,
                                            float &scale,
                                            float &max_vel)
  {
    // Modifica la velocidad para una escala determinada
    if (joy_msg->buttons[increment_vel])
    {

      // scale = scale*1.2; // Incremento de la escala
      scale = std::min(static_cast<double>(scale * 1.2), static_cast<double>(max_vel));
      ROS_INFO("Velocidad incrementada a %f", scale);
    }
    else if (joy_msg->buttons[decrement_vel])
    {

      // scale = std::max(scale/1.2, min_vel); // Decremento de la escala
      scale = std::max(static_cast<double>(scale / 1.2), static_cast<double>(min_vel));
      ROS_INFO("Velocidad decrementada a %f", scale);
    }
    ros::Duration(reaction_t).sleep(); // Espera un tiempo de reaccion
  }

  void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {

    if (joy_msg->buttons[enable_mov]) // Boton derecho
    {
      ROS_INFO("Boton LB pulsado");

      if (joy_msg->buttons[increment_vel] || joy_msg->buttons[decrement_vel])
      {
        // Subir velocidad
        ModifyVelocity(joy_msg, mov_vel, max_vel);
      }
      else
      {
        // Comandar velocidad
        ROS_INFO("Boton LB pulsado");
        cmd_vel_msg.linear.x = 0.1 * getVal(joy_msg, axis_position_map, "x");
        cmd_vel_msg.linear.y = 0.1 * getVal(joy_msg, axis_position_map, "y");
        // cmd_vel_msg.linear.z = 0.1 * getVal(joy_msg, axis_position_map, "z");

        cmd_vel_msg.angular.z = 0.1 * getVal(joy_msg, axis_orientation_map, "z");
        // cmd_vel_msg.angular.y = getVal(joy_msg, axis_orientation_map, "y");
        // cmd_vel_msg.angular.z = getVal(joy_msg, axis_orientation_map, "z");
        printTwistInfo(cmd_vel_msg, "Velocidad comandada");
      }
    }
    else
    { // Si no se toca LB -> Decelera
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.linear.x = 0.0;

      cmd_vel_msg.angular.x = 0.0;
      cmd_vel_msg.angular.y = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      printTwistInfo(cmd_vel_msg, "Velocidad comandada");
    }
    ROS_INFO("Publico velocidad");
    printTwistInfo(cmd_vel_msg, "Velocidad publicada");
    cmd_vel_pub.publish(cmd_vel_msg);
  }

} // namespace teleop_twist_joy
