#include "boost/assign.hpp"
//#include "quadnake_base/quadnake_hardware.h"

#include "../include/quadnake_base/quadnake_hardware.h"

namespace quadnake_base
{
QuadnakeHardware::QuadnakeHardware()
{
  ros::V_string leg_names = boost::assign::list_of("front_left_leg")("front_right_leg")("rear_left_leg")("rear_right_"
                                                                                                         "leg");

  for (int i = 0; i < leg_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(leg_names[i], &legs_[i].position, &legs_[i].velocity,
                                                            &legs_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &legs_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("drivefeed", 1, &QuadnakeHardware::feedbackCallback, this);
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

void QuadnakeHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);

  if (feedback_msg_lock && feedback_msg)
  {
    for (int i = 0; i < 4; i++)
    {
      legs_[i].position = feedback_msg->legs[i].position;
      legs_[i].effort = feedback_msg->legs[i].current;
    }
  }
}

void QuadnakeHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    // cmd_drive_pub_.msg_.legs[0].
    cmd_drive_pub_.msg_.mode = 1;  // for testing
    cmd_drive_pub_.msg_.legs[0].current = legs_[0].velocity_command;
    cmd_drive_pub_.unlockAndPublish();
  }
}

void QuadnakeHardware::feedbackCallback(const quadnake_msgs::Feed::ConstPtr &msg)
{
  boost::mutex::scoped_lock callbacklock(feedback_msg_mutex_);
  feedback_msg = msg;
}

}  // namespace quadnake_base