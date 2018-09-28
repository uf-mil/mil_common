#pragma once

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

namespace pcodar
{

class PersistentCloudFilter
{
public:
  PersistentCloudFilter();
  void filter(point_cloud& pc);
  void set_bounds(point_cloud bounds
private:
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
  point_cloud_ptr bounds;
};

} // namespace pcodar
