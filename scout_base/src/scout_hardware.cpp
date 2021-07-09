/*******************************************************************************
** MIT License
**
** Copyright(c) 2021 QuartzYan https://github.com/QuartzYan
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files(the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions :
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*******************************************************************************/

#include "scout_base/scout_hardware.h"

#include <boost/assign.hpp>

namespace scout_base
{
  ScoutHardware::ScoutHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
                                                      ("front_right_wheel")
                                                      ("rear_left_wheel")
                                                      ("rear_right_wheel");

    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, 
                                                              &joints_[i].velocity, 
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(joint_state_handle, 
                                                  &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    private_nh_.param<std::string>("scout_com", port_, "/dev/ttyUSB0");
    scout_ = new ScoutDriver(port_);

    light_cmd_sub_ = nh_.subscribe("scout_light_cmd", 1, &ScoutHardware::lightCmdCallBack, this);
    scout_status_pub_ = nh_.advertise<scout_msgs::ScoutStatus>("scout_status", 1);
  }

  ScoutHardware::~ScoutHardware()
  {
    scout_->~ScoutDriver();
    scout_ = NULL;
    delete scout_;
  }  

  // Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  void ScoutHardware::updateJointsFromHardware()
  {
    scout_msgs::ScoutStatus msg;
    ScoutDriver::ScoutStatus scout_status = scout_->getScoutStatus();

    msg.header.stamp = ros::Time::now();
    msg.base_state = scout_status.base_state;
    msg.control_mode = scout_status.control_mode;
    msg.error_code = scout_status.error_code;
    msg.battery_voltage = scout_status.battery_voltage;

    msg.vx = scout_status.vx;
    msg.vth = scout_status.vth;

    for(uint8_t i = 0; i < msg.motor_states.size(); i++)
    {
      msg.motor_states[i].id = scout_status.motor_states[i].id;
      msg.motor_states[i].rpm = scout_status.motor_states[i].rpm;
      msg.motor_states[i].current = scout_status.motor_states[i].current;
      msg.motor_states[i].encoder = scout_status.motor_states[i].encoder;
      msg.motor_states[i].motor_pose = scout_status.motor_states[i].motor_pose;
    }

    for(uint8_t i = 0; i < msg.driver_states.size(); i++)
    {
      msg.driver_states[i].id = scout_status.driver_states[i].id;
      msg.driver_states[i].driver_voltage = scout_status.driver_states[i].driver_voltage;
      msg.driver_states[i].driver_temperature = scout_status.driver_states[i].driver_temperature;
      msg.driver_states[i].motor_temperature = scout_status.driver_states[i].motor_temperature;
      msg.driver_states[i].driver_state = scout_status.driver_states[i].driver_state;
    }

    msg.light_state.enable = scout_status.light_state.enable;
    msg.light_state.front_light_mode = scout_status.light_state.front_light_mode;
    msg.light_state.front_light_brightness = scout_status.light_state.front_light_brightness;
    msg.light_state.rear_light_mode = scout_status.light_state.rear_light_mode;
    msg.light_state.rear_light_brightness = scout_status.light_state.rear_light_brightness;

    msg.odom.left_odom = scout_status.odom.left_odom;
    msg.odom.right_odom = scout_status.odom.right_odom;

    msg.version.controller_hw_version = scout_status.version.controller_hw_version;
    msg.version.controller_sw_version = scout_status.version.controller_sw_version;
    msg.version.driver_hw_version = scout_status.version.driver_hw_version;
    msg.version.driver_sw_version = scout_status.version.driver_sw_version;

    msg.teleop.swa = scout_status.teleop.swa;
    msg.teleop.swb = scout_status.teleop.swb;
    msg.teleop.swc = scout_status.teleop.swc;
    msg.teleop.swd = scout_status.teleop.swd;
    msg.teleop.right_rl_axis = scout_status.teleop.right_rl_axis;
    msg.teleop.right_ud_axis = scout_status.teleop.right_ud_axis;
    msg.teleop.left_ud_axis = scout_status.teleop.left_ud_axis;
    msg.teleop.left_rl_axis = scout_status.teleop.left_rl_axis;
    msg.teleop.left_vra = scout_status.teleop.left_vra;

    msg.bms.soc = scout_status.bms.soc;
    msg.bms.soh = scout_status.bms.soh;
    msg.bms.voltage = scout_status.bms.voltage;
    msg.bms.current = scout_status.bms.current;
    msg.bms.temperature = scout_status.bms.temperature;
       
    scout_status_pub_.publish(msg);

    static ros::Time now_time;
    static ros::Time last_time;

    now_time = ros::Time::now();

    if (last_time.isZero())
    {
      last_time = now_time;
      return;
    }

    double dt = (now_time - last_time).toSec();

    if (dt == 0)
      return;
    last_time = now_time;

    joints_[0].velocity = (scout_status.motor_states[3].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //front_left_wheel
    joints_[1].velocity = (scout_status.motor_states[0].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //front_right_wheel
    joints_[2].velocity = (scout_status.motor_states[2].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //rear_left_wheel
    joints_[3].velocity = (scout_status.motor_states[1].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //rear_right_wheel

    // joints_[0].position = scout_status.motor_states[3].motor_pose;
    // joints_[1].position = scout_status.motor_states[0].motor_pose;
    // joints_[2].position = scout_status.motor_states[2].motor_pose;
    // joints_[3].position = scout_status.motor_states[1].motor_pose;

    joints_[0].position += joints_[0].velocity * dt;
    joints_[1].position += joints_[1].velocity * dt;
    joints_[2].position += joints_[2].velocity * dt;
    joints_[3].position += joints_[3].velocity * dt;
  }

  // Get latest velocity commands from ros_control via joint structure, and send to MCU
  void ScoutHardware::writeCommandsToHardware()
  {
    float vx = (joints_[0].velocity_command + joints_[1].velocity_command) * WHEEL_RADIUS / 2.0;
    float vth = (joints_[1].velocity_command - joints_[0].velocity_command) * WHEEL_RADIUS * 0.5 / WHEEL_BASE;
    //std::cout << "vx:" << vx << "/n" << "vth:" << vth << std::endl;
    scout_->setSpeed(vx, vth);
  }

  void ScoutHardware::lightCmdCallBack(const scout_msgs::ScoutLightCmd::ConstPtr& msg)
  {
    std::map<uint8_t, uint8_t> map = {{0x00, 0x00},
                                      {0x01, 0x01},
                                      {0x02, 0x02},
                                      {0x03, 0x03}};

    if(msg->front_custom_value > 100 || msg->rear_custom_value > 100)
    {
      ROS_WARN("msg param error!!");
      return;
    }

    auto search1 = map.find(msg->front_mode);
    auto search2 = map.find(msg->rear_mode);

    if (search1 == map.end() || search2 == map.end())
    {
      ROS_WARN("msg param error!!");
      return;
    }

    // std::cout << search1->second << std::endl;
    // std::cout << search2->second << std::endl;
    
    scout_->setLightMode(search1->second, msg->front_custom_value, search2->second, msg->rear_custom_value);
  }

} // namespace scout_base
