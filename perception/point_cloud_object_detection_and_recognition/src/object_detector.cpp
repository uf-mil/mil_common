#include <point_cloud_object_detection_and_recognition/object_detector.hpp>
#include <tf/transform_datatypes.h>

namespace pcodar
{

mil_msgs::PerceptionObjectArrayPtr object_detector::get_objects(const point_cloud& accrued_cloud)
{
  mil_msgs::PerceptionObjectArrayPtr object_array(new mil_msgs::PerceptionObjectArray);
  point_cloud_ptr pcloud_ptr = accrued_cloud.makeShared();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // The radius around each point to check for neighbors in meters.
  ec.setClusterTolerance(params.cluster_tolerance_m);
  ec.setMinClusterSize(params.cluster_min_num_points);
  ec.setMaxClusterSize(params.cluster_max_num_points);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcloud_ptr);
  ec.extract(cluster_indices);

  std::vector<mil_msgs::PerceptionObject> objects;

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    std::vector<cv::Point2f> cv_points;
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    mil_msgs::PerceptionObject p_obj;

    p_obj.classification = "UNKNOWN";

    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
        pcl::PointXYZ p = pcloud_ptr->points[*pit];
        cv_points.emplace_back(p.x, p.y);
        if (p.z > max_z)
        {
            max_z = p.z;
        }

        if (p.z < min_z)
        {
            min_z = p.z;
        }
        geometry_msgs::Point32 g_point;
        g_point.x = p.x;
        g_point.y = p.y;
        g_point.z = p.z;

        p_obj.points.emplace_back(g_point);
    }
    cv::RotatedRect rect = cv::minAreaRect(cv_points);
    p_obj.header.frame_id = "enu";
    p_obj.header.stamp = ros::Time();
    p_obj.pose.position.x = rect.center.x;
    p_obj.pose.position.y = rect.center.y;
    p_obj.pose.position.z = (max_z + min_z) / 2.0;
    p_obj.scale.x = rect.size.width;
    p_obj.scale.y = rect.size.height;
    p_obj.scale.z = max_z - min_z;

    auto quat = tf::createQuaternionFromRPY(0.0, 0.0, rect.angle * 3.14159 / 180);
    quat.normalize();
    p_obj.pose.orientation.x = quat.x();
    p_obj.pose.orientation.y = quat.y();
    p_obj.pose.orientation.z = quat.z();
    p_obj.pose.orientation.w = quat.w();

    p_obj.id = objects.size() + 1;
    objects.emplace_back(p_obj);
  }

  object_array->objects = objects;
  return object_array;
}

} // namespace pcodar
