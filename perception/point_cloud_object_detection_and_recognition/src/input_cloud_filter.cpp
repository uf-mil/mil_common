#include <point_cloud_object_detection_and_recognition/input_cloud_filter.hpp>

namespace pcodar
{

InputCloudFilter::InputCloudFilter()
{
  bounds_filter_.setDim(2);
  bounds_filter_.setCropOutside(false);
}

void InputCloudFilter::set_bounds(point_cloud_ptr bounds)
{
  pcl::Vertices indicies;
  indicies.vertices.reserve(bounds->size());
  for (size_t i = 0; i < bounds->size(); ++i)
    indicies.vertices.push_back(i);
  std::vector<pcl::Vertices> hull = {indicies};

  bounds_filter_.setHullCloud(bounds);
  bounds_filter_.setHullIndices(hull);
}

void InputCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
  // Filter out bounds
  bounds_filter_.setInputCloud(in);
  bounds_filter_.filter(pc);
}

} // namespace pcodar
