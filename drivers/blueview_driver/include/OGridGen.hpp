#pragma once
#include <blueview_driver/BlueViewPing.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <stdexcept>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/circular_buffer.hpp>


class OGridGen {

public:
  OGridGen(ros::NodeHandle *nh);
  void publish_ogrid(const blueview_driver::BlueViewPingPtr &ping_msg);

  void callback(const blueview_driver::BlueViewPingPtr &ping_msg);

  ros::Publisher pubGrid;
  ros::NodeHandle *nh_;

  tf::TransformListener listener;

  ros::Publisher pubPointCloud;

  float ogrid_size;
  float resolution;
  float pool_depth;

  boost::circular_buffer<pcl::PointXYZI> pointCloudBuffer;

};