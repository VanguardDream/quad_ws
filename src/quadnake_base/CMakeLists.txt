cmake_minimum_required(VERSION 2.8.3)
project(quadnake_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  geometry_msgs
  quadnake_msgs
  roscpp
  roslaunch
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(simple_receiver src/simple_joy_receiver.cpp)

target_link_libraries(simple_receiver ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(simple_receiver quadnake_msgs_gencpp)

