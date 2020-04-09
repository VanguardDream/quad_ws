#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/asio/io_service.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <string>
#include <rosserial_server/serial_session.h>

#include "../include/quadnake_base/quadnake_hardware.h"
//#include "quadnake_base/quadnake_hardware.h"

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::Rate rate, quadnake_base::QuadnakeHardware *robot, controller_manager::ControllerManager *cm)
{
  time_source::time_point last_time = time_source::now();

  while (1)
  {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());

    last_time = this_time;

    robot->copyJointsFromHardware();
    cm->update(ros::Time::now(), elapsed);
    robot->publishDriveFromController();
    rate.sleep();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "quadnake_base_node");
  quadnake_base::QuadnakeHardware quadnake;

  std::string port_ = "/dev/quadnake";

  boost::asio::io_service io_service;

  new rosserial_server::SerialSession(io_service, port_, 115200);
  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  ros::NodeHandle nh_("");
  controller_manager::ControllerManager cm(&quadnake, nh_);

  boost::thread(boost::bind(controlThread, ros::Rate(50), &quadnake, &cm));

  ros::spin();

  return 0;
}
