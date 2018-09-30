#include <point_cloud_object_detection_and_recognition/persistent_cloud_filter.hpp>

namespace pcodar
{

PersistentCloudFilter::PersistentCloudFilter():
  outlier_filter_(false)
{
  outlier_filter_.setStddevMulThresh(params.outier_removal_std_dev_thresh);
  outlier_filter_.setMeanK(params.outier_removal_mean_k);
}

void PersistentCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
  outlier_filter_.setInputCloud(in);
  outlier_filter_.filter(pc);
}

} // namespace pcodar
