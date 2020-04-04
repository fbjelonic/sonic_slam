#include <ros/ros.h>
#include "sonic_slam/SonicSlam.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonic_slam");
  ros::NodeHandle nodeHandle("~");

  sonic_slam::SonicSlam sonic(nodeHandle);

  ros::spin();
  return 0;
}
