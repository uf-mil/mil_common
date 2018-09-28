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
namespace {
point_cloud filter(const point_cloud& in_cloud) {
  point_cloud out_cloud = in_cloud;
  const auto buffered_cloud_ptr_1 = out_cloud.makeShared();

  // Initializing with true will allow us to extract the removed indices
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
  sorfilter.setInputCloud(buffered_cloud_ptr_1);
  sorfilter.setStddevMulThresh(params.outier_removal_std_dev_thresh);
  sorfilter.setMeanK(params.outier_removal_mean_k);
  sorfilter.filter(out_cloud);

/* TODO: filter boat and bounds again
  // Store temp variables for discovering if point inside boundary
  auto b1 = boundary[0].head<2>();
  auto ab = b1 - boundary[1].head<2>();
  Eigen::Vector2d ac = b1 - boundary[2].head<2>();
  if (ab.dot(ac) > .1) {
    ac = b1 - boundary[3].head<2>();
  }

  // Begin point removal procedure
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < out_cloud.size(); i++) {
    pcl::PointXYZ pt(out_cloud.points[i].x, out_cloud.points[i].y,
                     out_cloud.points[i].z);
    Eigen::Vector2d am = b1 - Eigen::Vector2d(pt.x, pt.y);

    // Remove points outside boundary
    if (!(0 <= ab.dot(am) && ab.dot(am) <= ab.dot(ab) && 0 <= am.dot(ac) &&
          am.dot(ac) <= ac.dot(ac)))

    {
      inliers->indices.push_back(i);
    }
    // Remove points close the lidar
    if (abs(pt.x - e_velodyne_to_X.translation().x()) < params.remove_points_near_lidar_distance &&
        abs(pt.y - e_velodyne_to_X.translation().y()) < params.remove_points_near_lidar_distance) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(out_cloud.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(out_cloud);
*/
  return out_cloud;
}
}  // anonymous namespace

void point_cloud_builder::add_point_cloud(const point_cloud_ptr& pc) {
  // Add new cloud to buffer
  prev_clouds_.push_back(pc);

  // Don't contruct mega cloud until buffer of recent clouds is full
  if(!prev_clouds_.full())
    return;

  //  Assemple cloud as union of buffered clouds
  point_cloud buffered_cloud;
  for (const auto& cloud : prev_clouds_) {
      buffered_cloud += *cloud;
  }

  // Filter cloud, TODO: move to own class
  *mega_cloud_  = filter(buffered_cloud);
}

point_cloud_ptr point_cloud_builder::get_point_cloud() {
  // Returning
  return mega_cloud_;
}

}  // pcodar namespace
