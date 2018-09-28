#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>

namespace pcodar
{
class InputCloudFilter
{
public:
  InputCloudFilter();
  void filter(point_cloud& pc);

private:
  pcl::CropHull<pcl::PointXYZ> bounds_filter_;
  pcl::CropBox<pcl::PointXYZ> robot_filter_;
};

} // namespace pcodar
