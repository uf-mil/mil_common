#include <point_cloud_object_detection_and_recognition/object_detector.hpp>
#include <tf/transform_datatypes.h>

namespace pcodar
{

clusters_t object_detector::get_clusters(point_cloud_const_ptr pc)
{

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pc);

  clusters_t cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // The radius around each point to check for neighbors in meters.
  ec.setClusterTolerance(params.cluster_tolerance_m);
  ec.setMinClusterSize(params.cluster_min_num_points);
  ec.setMaxClusterSize(params.cluster_max_num_points);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.extract(cluster_indices);

  return cluster_indices;
}

} // namespace pcodar
