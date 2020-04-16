cmake_minimum_required(VERSION 2.8.3)
project(quadnake_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  quadnake_msgs
  roscpp
  roslaunch
  std_msgs
  realtime_tools
  sensor_msgs
  hardware_interface
  rosserial_server
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(quadnake_base_node
  src/quadnake_hardware.cpp
  src/quadnake_base_node.cpp
  )

target_link_libraries(quadnake_base_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(quadnake_base_node quadnake_msgs_gencpp rosserial_msgs_gencpp sensor_msgs)

add_executable(simple_receiver 
src/simple_joy_receiver.cpp
  )

target_link_libraries(simple_receiver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(simple_receiver quadnake_msgs_gencpp)

