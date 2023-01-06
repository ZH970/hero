//
// Created by zh on 22-12-6.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<
                                  hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void odom(int argc, char** arg, float x1, float y1, float z1, float odm,
            float vel_x, float vel_y, float w);

  hardware_interface::JointHandle front_left_joint_, front_right_joint_,
      back_left_joint_, back_right_joint_;

 private:
  ros::Time last_change_;
  ros::NodeHandle nh;
  control_toolbox::Pid pid_controller_;

  //Kinematic data：
  double exp_vel = 2.5;
  double vel = 0.0;  //Real vel
  double w = 1.0;    //Angular velocity of yaw axis
  double err = exp_vel - vel;
  double p = 0.5;
  double a =
      0.2;  //Transverse projection of the distance from the center of the robot chassis to the center of the wheel
  double b =
      0.2;  //Longitudinal projection of the distance from the center of the robot chassis to the center of the wheel

  float x = 0;   //Original x coordinate
  float y = 0;   //Original y coordinate
  float x1 = 0;  //To computer the Coordinate change rate
  float y1 = 0;
  double vel_x = 0;
  double vel_y = 0;
  float pose_x = 0;
  float pose_y = 0;
  float angel = 0;
  double odm = 0;

  double v1 = vel_y + vel_x - w * (a + b);
  double v2 = vel_y - vel_x + w * (a + b);
  double v3 = vel_y - vel_x - w * (a + b);
  double v4 = vel_y + vel_x + w * (a + b);
};
}  // namespace hero_chassis_controller

#endif  //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
