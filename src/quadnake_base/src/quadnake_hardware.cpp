#include "boost/assign.hpp"
#include "quadnake_base/quadnake_hardware.h"

#include "../include/quadnake_base/quadnake_hardware.h"


namespace quadnake_base
{

QuadnakeHardware::QuadnakeHardware()
{
    ros::V_string leg_names = boost::assign::list_of("front_left_leg")("front_right_leg")("rear_left_leg")("rear_right_leg");

    for(int i = 0 ; i < leg_names.size() ; i ++)
    {
        hardware_interface::JointStateHandle joint_state_handle(leg_names[i], &legs_[i].position, &legs_[i].velocity, &legs_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle, &legs_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    
}

}

int main()
{

}