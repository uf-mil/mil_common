#include <point_cloud_object_detection_and_recognition/ogrid_manager.hpp>

namespace pcodar
{

void ogrid_manager::initialize(ros::NodeHandle& nh) {
  pub_ogrid_ = nh.advertise<nav_msgs::OccupancyGrid>("/ogrid", 5);

  ogrid_.header.frame_id = "enu";
  ogrid_.info.resolution = resolution_meters_per_cell_;
  ogrid_.info.width = resolution_meters_per_cell_ * width_meters_;
  ogrid_.info.height = resolution_meters_per_cell_ * height_meters_;
  ogrid_.info.origin.position.x = -10000*0.09/2;
  ogrid_.info.origin.position.y = -10000*0.09/2;
  ogrid_.info.origin.orientation.w = 1;
  ogrid_.data.resize(ogrid_.info.width * ogrid_.info.height);

  ogrid_mat_ = cv::Mat(cv::Size(ogrid_.info.width, ogrid_.info.height), CV_8UC1, ogrid_.data.data());
}

void ogrid_manager::draw_boundary() {
/*
  std::vector<cv::Point>bounds(pcodar::boundary.size());
  for(int i = 0; i < bounds.size(); ++i) {
    bounds[i] = point_in_ogrid(bounds[i]);
  }

  for(int i = 0; i < bounds.size(); ++i) {
    // std::cout << bounds[i] << std::endl;
    cv::circle(ogrid_mat_, bounds[i], 15, cv::Scalar(99), -1);
  }
  const cv::Point *pts = (const cv::Point*) cv::Mat(bounds).data;
  int npts = cv::Mat(bounds).rows;

  cv::polylines(ogrid_mat_, &pts, &npts, 1, true, cv::Scalar(99), 5);
*/
}

cv::Point ogrid_manager::point_in_ogrid(point_t point)
{
  return cv::Point(point.x / resolution_meters_per_cell_ + ogrid_.info.width/2,
                   point.y / resolution_meters_per_cell_ + + ogrid_.info.height/2);
}

void ogrid_manager::update_ogrid(ObjectMap const& objects)
{
  // Clear ogrid
  ogrid_mat_ = cv::Scalar(0);

  // Draw border on ogrid
  draw_boundary();

  for (auto const& pair : objects.objects_)
  {
    Object const& object = pair.second;
    for(const auto &point : object.points_) {
      cv::Point center(point_in_ogrid(point));
      cv::circle(ogrid_mat_, center, params.ogrid_inflation_cell, cv::Scalar(99), -1);
    }
  }

  ogrid_.header.stamp = ros::Time::now();

  pub_ogrid_.publish(ogrid_);
}

} // namespace pcodar
