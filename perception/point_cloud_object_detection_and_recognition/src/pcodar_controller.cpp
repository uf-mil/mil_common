#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>

#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>



#include <eigen_conversions/eigen_msg.h>

#include <ros/callback_queue.h>
#include <ros/console.h>

#include <chrono>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace pcodar {

pcodar_controller::pcodar_controller(ros::NodeHandle _nh)
    : nh_(_nh),
      bounds_client_("/bounds_server", std::bind(&pcodar_controller::bounds_update_cb, this, std::placeholders::_1)),
      tf_listener(tf_buffer_, nh_),
      global_frame_("enu") {
  highest_id_ = 0;
  ros::NodeHandle private_nh("~");
  set_params(private_nh);
  id_label_map_= std::make_shared<id_label_map>();
}

void pcodar_controller::initialize() {
  marker_manager_.initialize(nh_, id_label_map_);
  ogrid_manager_.initialize(nh_);

  modify_classification_service_ = nh_.advertiseService(
      "/database/requests", &pcodar_controller::DBQuery_cb, this);

  // Subscribe to odom and the velodyne
  pc_sub = nh_.subscribe("/velodyne_points", 1, &pcodar_controller::velodyne_cb,
                         this);
  odom_sub = nh_.subscribe("/odom", 1, &pcodar_controller::odom_cb, this);

  // Publish occupancy grid and visualization markers
  pub_pcl_ = nh_.advertise<point_cloud>("persist_pcl", 1);

  // Publish PerceptionObjects
  pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("objects", 1);
}

void pcodar_controller::velodyne_cb(
    const sensor_msgs::PointCloud2ConstPtr &pcloud) {
  point_cloud_ptr pc = boost::make_shared<point_cloud>();
  // Transform new pointcloud to ENU
  if (!transform_point_cloud(*pcloud, *pc)) return;

  // Filter out bounds / robot
  point_cloud_ptr filtered_pc = boost::make_shared<point_cloud>();
  input_cloud_filter_.filter(pc, *filtered_pc);

  // Add pointcloud to persistent cloud
  persistent_cloud_builder_.add_point_cloud(filtered_pc);

  // Get persistent cloud and publish for debug
  auto accrued = persistent_cloud_builder_.get_point_cloud();
  if ((*accrued).empty()) return;
  (*accrued).header.frame_id = "enu";
  pub_pcl_.publish(accrued);

  // Filter out outliers
  point_cloud_ptr filtered_accrued = boost::make_shared<point_cloud>();
  persistent_cloud_filter_.filter(accrued, *filtered_accrued);

  // Get object clusters from persistent pointcloud
  clusters_t clusters = detector_.get_clusters(filtered_accrued);

  // Associate current clusters with old ones
  ass.associate(objects_, *filtered_accrued, clusters);

  // TODO: reenable
  //marker_manager_.update_markers(objects_);
  //ogrid_manager_.update_ogrid(objects_, latest_odom_);

  pub_objects_.publish(objects_.to_msg());
}

void pcodar_controller::odom_cb(const nav_msgs::OdometryConstPtr &odom) {
  latest_odom_ = odom;
}

bool pcodar_controller::bounds_update_cb(const mil_bounds::BoundsConfig &config) {
  ROS_INFO("Updating bounds...");

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        "enu", config.frame,
        ros::Time::now(),
        ros::Duration(1, 0));  // change time to pcloud header? pcloud->header.stamp
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d e_transform = tf2::transformToEigen(transform);

  point_cloud_ptr bounds;
  bounds->push_back(point_t(config.x1, config.y1, config.z1));
  bounds->push_back(point_t(config.x2, config.y2, config.z2));
  bounds->push_back(point_t(config.x3, config.y3, config.z3));
  bounds->push_back(point_t(config.x4, config.y4, config.z4));

  pcl::transformPointCloud(*bounds, *bounds, e_transform);
  input_cloud_filter_.set_bounds(bounds);

  ROS_INFO("bounds updateded");
}

bool pcodar_controller::DBQuery_cb(mil_msgs::ObjectDBQuery::Request &req,
                                  mil_msgs::ObjectDBQuery::Response &res) {
/*
  if (req.cmd != "") {
    int pos = req.cmd.find_first_of("=");

    int id = -1;
    try {
      id = std::stoi(req.cmd.substr(0, pos));
    } catch (...) {
      std::cout << "Could not find id " << id << std::endl;
      res.found = false;
      return false;
    }
    std::string cmd = req.cmd.substr(pos + 1);
    auto it = id_object_map_->find(id);
    if (it == id_object_map_->end()) {
      std::cout << "Could not find id " << id << std::endl;
      return false;
    } else {
      std::cout << "set " << id << " to " << cmd << std::endl;
      it->second.labeled_classification = cmd;
    }
  }
  std::vector<mil_msgs::PerceptionObject> objects(id_object_map_->size());
  int i = 0;
  for (auto &o : *id_object_map_) {
    objects[i] = o.second;
    i++;
  }
  if (req.name == "all") {
    res.found = true;
    res.objects = objects;
    return true;
  }

  // if (std::find(classification_strings.begin(), classification_strings.end(),
                // req.name) != classification_strings.end()) {
  if (req.name != "")
    for (const auto &object : objects) {
      if (object.classification == req.name ||
          object.labeled_classification == req.name) {
        res.found = true;
        res.objects.push_back(object);
      }
    }
  // }
  return true;
*/
}

bool pcodar_controller::transform_point_cloud(const sensor_msgs::PointCloud2& pc_msg, point_cloud& out) {
  geometry_msgs::TransformStamped T_enu_velodyne_ros;
  try {
    T_enu_velodyne_ros = tf_buffer_.lookupTransform(
        global_frame_, pc_msg.header.frame_id,
        pc_msg.header.stamp,
        ros::Duration(1, 0));  // change time to pcloud header? pcloud->header.stamp
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Eigen::Affine3d e_transform = tf2::transformToEigen(T_enu_velodyne_ros);

  // Transform from PCL2
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(pc_msg, pcl_pc2);
  point_cloud pcloud;
  pcl::fromPCLPointCloud2(pcl_pc2, pcloud);

  out.clear();
  pcl::transformPointCloud(pcloud, out, e_transform);
  return true;
}

}  // namespace pcodar
