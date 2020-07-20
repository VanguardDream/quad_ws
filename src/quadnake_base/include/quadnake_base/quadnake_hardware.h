#ifndef __QUADNAKE_BASE_QUADNAKE_HARDWARE_H
#define __QUADNAKE_BASE_QUADNAKE_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
//#include "quadnake_msgs/Drive.h"
#include "quadnake_msgs/DriveFeed.h"
#include "quadnake_msgs/Feed.h"
#include "quadnake_msgs/LegsDrive.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"

namespace quadnake_base
{
class QuadnakeHardware : public hardware_interface::RobotHW
{
public:
  QuadnakeHardware();
  void copyJointsFromHardware();
  void publishDriveFromController();

private:
  void feedbackCallback(const quadnake_msgs::Feed::ConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber joy_sub_;

  realtime_tools::RealtimePublisher<quadnake_msgs::LegsDrive> cmd_drive_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  hardware_interface::JointCommandInterface joint_cmd_interface;  // for testing...

  // These are mutated on the controls thread only.
  struct Leg
  {
    /*
    double s_position;
    double f_position;
    double velocity;
    double curve;
    double duty;

    Leg() : s_position(0), f_position(0), velocity(0), curve(0), duty(0)
    {
    }
    */
    // for sensing
    double position;
    double velocity;
    double amplitude;
    double duty;

    // for commanding
    unsigned char motion_mode;
    unsigned char n_trough;
    double velocity_command;
    double position_command;
    double amplitude_command;

    Leg()
      : position(0)
      , velocity(0)
      , amplitude(0)
      , duty(0)
      , velocity_command(0)
      , position_command(0)
      , amplitude_command(0)
      , n_trough(0)
    {
    }
  } legs_[4];

  struct joy
  {
    double fb_linear;
    double lr_linear;
    double angular;

    joy() : fb_linear(0), lr_linear(0), angular(0)
    {
    }
  } joy;

  // This pointer is set from the ROS thread.
  quadnake_msgs::Feed::ConstPtr feedback_msg;
  boost::mutex feedback_msg_mutex_;
  boost::mutex joy_msg_mutex_;
};

}  // namespace quadnake_base

#endif  // __QUADNAKE_BASE_QUADNAKE_HARDWARE_H