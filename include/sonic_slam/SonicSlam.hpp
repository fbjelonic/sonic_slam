#pragma once

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"

#include <vector>

namespace sonic_slam {

class SonicSlam {
public:
  SonicSlam(ros::NodeHandle& nodeHandle);
  virtual ~SonicSlam();
  void sonicCB(const geometry_msgs::Point32 &msg);
private:
  void initMarker();
  void initLaserScaner();
  void trySth();
  void convert_to_scan();
  ros::NodeHandle nodeHandle_;

  std::vector<std_msgs::Float32> ranges_;
  visualization_msgs::Marker markerTemplate_;
  sensor_msgs::LaserScan laserScan_;
  ros::Subscriber sonicSub_;
  ros::Publisher markerPub_;
  ros::Publisher scanPub_;
};

}
