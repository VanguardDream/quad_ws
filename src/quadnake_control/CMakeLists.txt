cmake_minimum_required(VERSION 2.8.3)
project(quadnake_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  joy
  teleop_twist_joy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadnake_control
#  CATKIN_DEPENDS controller_manager interactive_marker_twist_server joint_state_controller joy teleop_twist_joy
#  DEPENDS system_lib
)
