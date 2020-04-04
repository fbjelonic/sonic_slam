#include "sonic_slam/SonicSlam.hpp"

namespace sonic_slam {

SonicSlam::SonicSlam(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  initMarker();
  initLaserScaner();
  sonicSub_ = nodeHandle_.subscribe("/sonic",10, &SonicSlam::sonicCB, this);
  markerPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("rviz_marker", 100);
  scanPub_ = nodeHandle_.advertise<sensor_msgs::LaserScan>("/scan", 100);
}

void SonicSlam::initMarker()
{
  markerTemplate_.type = visualization_msgs::Marker::POINTS; // visualize points
  markerTemplate_.header.frame_id = "/map"; // frame_id
  markerTemplate_.scale.x = 0.01; // points size
  markerTemplate_.scale.y = 0.01; // points size
  markerTemplate_.action = visualization_msgs::Marker::ADD; // modifiy or add
  markerTemplate_.id = 0; // id
  markerTemplate_.color.r = 1.0; // red points
  markerTemplate_.color.a = 1.0; // opaque
  markerTemplate_.pose.position.x = 0.0;
  markerTemplate_.pose.position.y = 0.0;
  markerTemplate_.pose.position.z = 0.0;
  markerTemplate_.points.push_back(geometry_msgs::Point());
  markerTemplate_.points.resize(180 / 3 + 1); // 0 - 180 degree
}

void SonicSlam::initLaserScaner()
{
  laserScan_.header.frame_id = "/laser";
  laserScan_.angle_min = 0.0;
  laserScan_.angle_max = 3.1415;
  laserScan_.angle_increment = 3.0 / 180 * 3.1415;
  laserScan_.time_increment = static_cast<float>(0.02);
  laserScan_.scan_time = 3.5;
  laserScan_.range_min = static_cast<float>(0.02);
  laserScan_.range_max = 0.4;
//  ROS_ERROR("Scan range: %d", laserScan_.ranges.size());
  laserScan_.ranges.resize(180 / 3 + 1);
  laserScan_.header.stamp = ros::Time::now();
}

SonicSlam::~SonicSlam()
{
}

void SonicSlam::sonicCB(const geometry_msgs::Point32 &msg)
{
//  trySth();
  geometry_msgs::Point point;
  markerTemplate_.header.stamp = ros::Time::now();
  if (msg.x > laserScan_.range_max) {
    markerTemplate_.points.at(static_cast<size_t>(msg.z/3)) = point;
    markerPub_.publish(markerTemplate_);
    return;
  }
  laserScan_.header.stamp = ros::Time::now();
  laserScan_.ranges.at(static_cast<size_t>(msg.z/3)) = static_cast<float>(msg.z);
  point.x = msg.x * cos(msg.z/180 * 3.1415);
  point.y = msg.x * sin(msg.z/180 * 3.1415);
  point.z = 0.1;
  markerTemplate_.points.at(static_cast<size_t>(msg.z/3)) = point;

  markerPub_.publish(markerTemplate_);
  scanPub_.publish(laserScan_);
}

} /* namespace */
