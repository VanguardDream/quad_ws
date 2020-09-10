cmake_minimum_required(VERSION 2.8.3)
project(quadnake_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  RemoteDrive.msg
  LegsDrive.msg
  LegCommand.msg
  Feed.msg
  DriveFeed.msg
  MotorFeed.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  quadnake_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)