// This node receive Joy topic and publish quadnake_msgs to control robot.

#include "quadnake_msgs/RemoteDrive.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "boost/algorithm/clamp.hpp"

class quad_receiver
{
public:
  explicit quad_receiver(ros::NodeHandle* nh);

private:
  void joycallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

  ros::NodeHandle* _nh;
  ros::Subscriber _sub;
  ros::Publisher _pub;
};

quad_receiver::quad_receiver(ros::NodeHandle* nh) : _nh(nh)
{
  _sub = _nh->subscribe<sensor_msgs::Joy>("/joystick/joy", 1, &quad_receiver::joycallback, this);
  _pub = _nh->advertise<quadnake_msgs::RemoteDrive>("cmd_drive", 1, true);
}

void quad_receiver::joycallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  quadnake_msgs::RemoteDrive sending_msg;

  // select gait mode from joy
  int q_mode = 0;  // = joy_msg->buttons[0];

  for (int idx = 0; idx <= 3; idx++)
  {
    if (joy_msg->buttons[idx])
      q_mode = idx + 1;
  }

  // make some value to drive robot
  float fb_linear = joy_msg->axes[1] * 100.0f;
  float lr_linear = joy_msg->axes[0] * -100.0f;
  float angular = joy_msg->axes[3] * -100.0f;

  // make some value to adjust platform

  // publish msg
  sending_msg.MODE = (int8_t)q_mode;
  sending_msg.FORWARD_DRIVE = (int8_t)fb_linear;
  sending_msg.SIDE_DRIVE = (int8_t)lr_linear;
  sending_msg.ANGULAR_DRIVE = (int8_t)angular;

  _pub.publish(sending_msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "quadnake_joy_receiver");

  ros::NodeHandle nh;
  quad_receiver qr(&nh);

  ros::spin();
}