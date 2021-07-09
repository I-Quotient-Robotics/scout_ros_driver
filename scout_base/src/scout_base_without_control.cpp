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
#include <map>
#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/assign.hpp>

#include "scout_msgs/ScoutLightCmd.h"
#include "scout_msgs/ScoutStatus.h"
#include "ScoutDriver.h"

class ScoutDriverNode
{
public:
  ScoutDriverNode(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh)
  {
    joints_.name.resize(4);
    joints_.position.resize(4);
    joints_.velocity.resize(4);
    joints_.effort.resize(4);
    joints_.name[0] = "front_left_wheel";
    joints_.name[1] = "front_right_wheel";
    joints_.name[2] = "rear_left_wheel";
    joints_.name[3] = "rear_right_wheel";

    private_nh_.param<std::string>("scout_com", port_, "/dev/ttyUSB0");
    private_nh_.param<bool>("enable_odom_tf", enable_odom_tf_, true);
    scout_ = new ScoutDriver(port_);

    vel_cmd_sub_ = nh_.subscribe("cmd_vel", 1, &ScoutDriverNode::velCmdCallBack, this);
    light_cmd_sub_ = nh_.subscribe("scout_light_cmd", 1, &ScoutDriverNode::lightCmdCallBack, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
    joint_Pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    scout_status_pub_ = nh_.advertise<scout_msgs::ScoutStatus>("scout_status", 1);

    std::vector<double> pose_cov_diag(6, 1e-6);
    std::vector<double> twist_cov_diag(6, 1e-6);

    pose_covariance_ = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)
                                              (0)(pose_cov_diag[1])(0)(0)(0)(0)
                                              (0)(0)(pose_cov_diag[2])(0)(0)(0)
                                              (0)(0)(0)(pose_cov_diag[3])(0)(0)
                                              (0)(0)(0)(0)(pose_cov_diag[4])(0)
                                              (0)(0)(0)(0)(0)(pose_cov_diag[5]);
    twist_covariance_ = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)
                                              (0)(twist_cov_diag[1])(0)(0)(0)(0)
                                              (0)(0)(twist_cov_diag[2])(0)(0)(0)
                                              (0)(0)(0)(twist_cov_diag[3])(0)(0)
                                              (0)(0)(0)(0)(twist_cov_diag[4])(0)
                                              (0)(0)(0)(0)(0)(twist_cov_diag[5]);

    boost::thread(boost::bind(&ScoutDriverNode::update, this));
  };
  ~ScoutDriverNode()
  {
    scout_->~ScoutDriver();
    scout_ = NULL;
    delete scout_;
  };

private:
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber light_cmd_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher joint_Pub_;
  ros::Publisher scout_status_pub_;

  std::string port_;
  bool enable_odom_tf_;

  ScoutDriver *scout_;
  sensor_msgs::JointState joints_;

  tf2_ros::TransformBroadcaster odom_broadcaster_;

  boost::assign_detail::generic_list<double> pose_covariance_;
  boost::assign_detail::generic_list<double> twist_covariance_;

  void lightCmdCallBack(const scout_msgs::ScoutLightCmd::ConstPtr& msg)
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
  };
  void velCmdCallBack(const geometry_msgs::Twist::ConstPtr& msg)
  {
    //std::cout << "vx:" << vx << "/n" << "vth:" << vth << std::endl;
    scout_->setSpeed(msg->linear.x, msg->angular.z);
  };

public:
  void update()
  {
    ros::Rate rate(50);
    while (1)
    {
      //pub scout status
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
        continue;
      }

      double dt = (now_time - last_time).toSec();

      if (dt == 0)
        continue;
      last_time = now_time;

      //pub joint
      joints_.header.stamp = ros::Time::now();
      joints_.velocity[0] = (scout_status.motor_states[3].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //front_left_wheel
      joints_.velocity[1] = (scout_status.motor_states[0].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //front_right_wheel
      joints_.velocity[2] = (scout_status.motor_states[2].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //rear_left_wheel
      joints_.velocity[3] = (scout_status.motor_states[1].rpm / (60.0 * REDUCTION_RATIO)) * 3.1415926 * 2; //rear_right_wheel
      // joints_.position[0] = scout_status.motor_states[1].motor_pose;
      // joints_.position[1] = scout_status.motor_states[0].motor_pose;
      // joints_.position[2] = scout_status.motor_states[2].motor_pose;
      // joints_.position[3] = scout_status.motor_states[3].motor_pose;
      joints_.position[0] += joints_.velocity[0] * dt;
      joints_.position[1] += joints_.velocity[1] * dt;
      joints_.position[2] += joints_.velocity[2] * dt;
      joints_.position[3] += joints_.velocity[3] * dt;
      joint_Pub_.publish(joints_);

      //pub odom
      static double x = 0.0;
      static double y = 0.0;
      static double th = 0.0;

      double vr = (joints_.velocity[1] + joints_.velocity[3]) * WHEEL_RADIUS / 2.0;
      double vl = (joints_.velocity[0] + joints_.velocity[2]) * WHEEL_RADIUS / 2.0;

      double vx = (vr + vl) / 2.0;
      double vth = 0.5 * (vr - vl) / WHEEL_BASE;

      //std::cout << vth << std::endl;

      //calculate dx dy dth
      double delta_x = (vx * cos(th)) * dt;
      double delta_y = (vx * sin(th)) * dt;
      double delta_th = vth * dt;
      x += delta_x;
      y += delta_y;
      th += delta_th;

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = now_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_footprint";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      tf2::Quaternion odom_quat;
      odom_quat.setRPY(0, 0, th);
      odom_trans.transform.rotation.x = odom_quat.x();
      odom_trans.transform.rotation.y = odom_quat.y();
      odom_trans.transform.rotation.z = odom_quat.z();
      odom_trans.transform.rotation.w = odom_quat.w();

      //send the transform
      if (enable_odom_tf_)
      {
        odom_broadcaster_.sendTransform(odom_trans);
      }

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = now_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = odom_quat.x();
      odom.pose.pose.orientation.y = odom_quat.y();
      odom.pose.pose.orientation.z = odom_quat.z();
      odom.pose.pose.orientation.w = odom_quat.w();
      //odom.pose.pose.orientation = odom_quat;
      odom.pose.covariance = pose_covariance_;
      //set the velocity
      odom.child_frame_id = "base_footprint";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = vth;
      odom.twist.covariance = twist_covariance_;

      odom_pub_.publish(odom);

      rate.sleep();
    }
  };
};



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scout_base_without_control");
  ros::NodeHandle nh, private_nh("~");

  ScoutDriverNode sc(nh, private_nh);

  ros::spin();

  return 0;
}