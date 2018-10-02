#include <point_cloud_object_detection_and_recognition/object.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <tf2/LinearMath/Quaternion.h>


namespace pcodar
{

Object::Object(point_cloud const& _pc):
  points_(_pc)
{
}

void Object::update_points(point_cloud const& pc)
{
  points_ = pc;
}

mil_msgs::PerceptionObject Object::to_msg()
{
  std::vector<cv::Point2f> cv_points;
  double min_z = std::numeric_limits<double>::max();
  double max_z = -std::numeric_limits<double>::max();

  mil_msgs::PerceptionObject p_obj;

  p_obj.classification = "UNKNOWN";

  for (point_t const& point: points_)
  {
      cv_points.emplace_back(point.x, point.y);
      if (point.z > max_z)
      {
          max_z = point.z;
      }

      if (point.z < min_z)
      {
          min_z = point.z;
      }
      geometry_msgs::Point32 g_point;
      g_point.x = point.x;
      g_point.y = point.y;
      g_point.z = point.z;

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

  tf2::Quaternion quat;
  quat.setRPY(0., 0., rect.angle * 3.14159 / 180);
  quat.normalize();
  p_obj.pose.orientation.x = quat.x();
  p_obj.pose.orientation.y = quat.y();
  p_obj.pose.orientation.z = quat.z();
  p_obj.pose.orientation.w = quat.w();
  return p_obj;
}


} // namespace pcodar
