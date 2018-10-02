#pragma once

#include "pcodar_types.hpp"
#include "pcodar_params.hpp"
#include "object_map.hpp"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace pcodar
{

class ogrid_manager
{
  public:
    void initialize(ros::NodeHandle& nh);
    void update_ogrid(ObjectMap const& objects);
    void draw_boundary();
  private:
    double resolution_meters_per_cell_ = 0.3;
    cv::Point point_in_ogrid(point_t point);
    uint32_t width_meters_ = 1000;
    uint32_t height_meters_ = 1000;
    ros::Publisher pub_ogrid_;
    cv::Mat ogrid_mat_;
    nav_msgs::OccupancyGrid ogrid_;
};

} // namespace pcodar
