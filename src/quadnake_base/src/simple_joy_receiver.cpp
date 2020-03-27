//This node receive Joy topic and publish quadnake_msgs to control robot.

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "quadnake_msgs/Drive.h"

#include "boost/algorithm/clamp.hpp"

class quad_receiver {

    public:
        explicit quad_receiver( ros::NodeHandle* nh );

    private:
        void joycallback( const sensor_msgs::Joy::ConstPtr& joy_msg );

        ros::NodeHandle* _nh;
        ros::Subscriber _sub;
        ros::Publisher _pub;

};

quad_receiver::quad_receiver(   ros::NodeHandle* nh   ) : _nh(nh) {
    
    _sub = nh->subscribe<sensor_msgs::Joy>("joy",1, &quad_receiver::joycallback);
    _pub = nh->advertise<quadnake_msgs::Drive>("cmd_drive",1,true);

}

void quad_receiver::joycallback( const sensor_msgs::Joy::ConstPtr& joy_msg )
{
    //select gait mode from joy
    int q_mode = joy_msg->buttons[0];

    //make some value to drive robot
    float fb_linear = joy_msg->axes[1] * 1.0f;
    float lr_linear = joy_msg->axes[2] * 1.0f;
    float angular = joy_msg->axes[0] * 1.0f;

    //make some value to adjust platform

}

int main (  int args, char* arge[]  )
{
    return 0;
}