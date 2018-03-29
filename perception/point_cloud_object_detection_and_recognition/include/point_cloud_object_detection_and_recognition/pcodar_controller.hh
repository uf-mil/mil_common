#pragma once

#include "marker_manager.hh"
#include "object_detector.hh"
#include "pcodar_params.hh"
#include "pcodar_types.hh"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/circular_buffer.hpp>

namespace pcodar
{
using point_cloud = pcl::PointCloud<pcl::PointXYZ>;
using point_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class pcodar_controller
{
   public:
    pcodar_controller(int argc, char *argv[]);

    void velodyne_cb(const sensor_msgs::PointCloud2ConstPtr& pcloud);

    void odom_cb(const nav_msgs::OdometryConstPtr &odom);

    void initialize();

    void execute();

    void executive();

   private:
    ros::NodeHandle nh_;
    pcodar_params params_;

    // Publishers
    ros::Publisher pub_grid_;
    ros::Publisher pub_objects_;
    ros::Publisher pub_pcl_;
    ros::Publisher pub_pcl_old_;

    // Subscriber
    ros::Subscriber pc_sub;
    ros::Subscriber odom_sub;

    // Place to hold the latest message
    sensor_msgs::PointCloud2 latest_point_cloud_;
    nav_msgs::OdometryConstPtr latest_odom_;

    // Visualization
    marker_manager marker_manager_;

    // Model (It eventually will be obejct tracker, but for now just detections)
    object_detector detector_;

};

}  // namespace pcodar