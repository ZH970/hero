//
// Created by zh on 22-12-6.
//

#include "hero_chassis_controller.h"
#include <control_toolbox/pid.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pluginlib/class_list_macros.hpp>
#include "gazebo_msgs/GetModelState.h"

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {

  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");

  return true;
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {

  const control_toolbox::Pid pid;

  ros::ServiceClient client2 =
      nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState srv2;
  srv2.request.model_name = "hero";  //Assign robot name

  if (client2.call(srv2)) {
    // srv2.response is the data that we call
    pose_x = srv2.response.pose.position.x;
    pose_y = srv2.response.pose.position.y;
    angel = srv2.response.twist.angular.x;  //Angle between speed and x-axis

  } else {
    ROS_ERROR("Failed to call service /gazebo/get_model_state");
  }
  pid_controller_ = pid;

  if ((time - last_change_).toSec() > 2) {

    x1 = pose_x;
    y1 = pose_y;
    vel_x =
        (x1 - x) /
        (time - last_change_).toSec();  //"toSec" ouput double. But x is float.
    vel_y = (y1 - y) / (time - last_change_).toSec();
    vel = pow(pow(vel_x, 2) + pow(vel_y, 2), 0.5);  //Real vel

    x = x1;
    y = y1;

    v1 = vel_y + vel_x - w * (a + b);
    v2 = vel_y - vel_x + w * (a + b);
    v3 = vel_y - vel_x - w * (a + b);
    v4 = vel_y + vel_x + w * (a + b);
    odm = odm + (vel * (time - last_change_).toSec());
    ROS_INFO("Odom: %f", odm);

    if (err > 0.1) {
      double commanded_effort = pid_controller_.computeCommand(err, period);
      err = commanded_effort;
      vel = vel + err;
    }

    vel_x = vel * cos(angel);
    vel_y = vel * sin(angel);
    last_change_ = time;
  }

  static double cmd_[6][4] = {{v1, v2, v3, v4}};

  front_left_joint_.setCommand(cmd_[0][0]);
  front_right_joint_.setCommand(cmd_[0][1]);
  back_left_joint_.setCommand(cmd_[0][2]);
  back_right_joint_.setCommand(cmd_[0][3]);
}

void HeroChassisController::odom(int argc, char** argv, float x1, float y1,
                                 float z1, float odm, float vel_x, float vel_y,
                                 float w) {
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  float x = x1;
  float y = y1;
  float th = z1;

  float vx = vel_x;
  float vy = vel_y;
  float vth = w;

  ros::Time current_time;
  current_time = ros::Time::now();

  while (n.ok()) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);
  }
}
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
}  // namespace hero_chassis_controller
