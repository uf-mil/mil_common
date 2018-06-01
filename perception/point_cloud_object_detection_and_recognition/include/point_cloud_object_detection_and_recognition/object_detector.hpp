#pragma once

#include "pcodar_params.hpp"

#include "point_cloud_builder.hpp"

#include <mil_msgs/PerceptionObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace pcodar
{
class object_detector
{
   public:
    object_detector() {};
    mil_msgs::PerceptionObjectArrayPtr get_objects(ros::Publisher &pub_pcl_);
    void add_point_cloud(const sensor_msgs::PointCloud2& pcloud2, const Eigen::Affine3d& e_velodyne_to_enu);


   private:
    point_cloud_builder pc_builder_;
};
}
