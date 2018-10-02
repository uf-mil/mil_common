#include <point_cloud_object_detection_and_recognition/point_cloud_builder.hpp>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2/convert.h>

namespace pcodar {

point_cloud_builder::point_cloud_builder() 
 : prev_clouds_(params.number_persistant_point_clouds),
   mega_cloud_(boost::make_shared<point_cloud>())
{

}

void point_cloud_builder::add_point_cloud(const point_cloud_ptr& pc)
{
  // Add new cloud to buffer
  prev_clouds_.push_back(pc);

  // Don't contruct mega cloud until buffer of recent clouds is full
  if(!prev_clouds_.full())
    return;

  //  Assemple cloud as union of buffered clouds
  mega_cloud_->clear();
  for (const auto& cloud : prev_clouds_) {
      *mega_cloud_ += *cloud;
  }
}

point_cloud_ptr point_cloud_builder::get_point_cloud()
{
  return mega_cloud_;
}

}  // pcodar namespace
