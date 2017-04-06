#include "OGridGen.hpp"

OGridGen::OGridGen(ros::NodeHandle *nh) : pointCloudBuffer(5000) {
  nh_ = nh;
  pubGrid = nh_->advertise<nav_msgs::OccupancyGrid>("/ogrid", 10, true);
  pubPointCloud = nh_->advertise<sensor_msgs::PointCloud2> ("/point_cloud", 1);
  nh_->param<float>("resolution", resolution, 0.2);
  nh_->param<float>("ogrid_size", ogrid_size, 50.f);
  nh_->param<float>("pool_depth", pool_depth, 7.f);
}

void OGridGen::publish_ogrid(const blueview_driver::BlueViewPingPtr &ping_msg) {
  
  //Probably should wait for TF... 

  //TF for sonar ain't in bags
  tf::StampedTransform transform;
    try{
       listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      // std::cout << "failed" << std::endl;
      ros::Duration(1.0).sleep();
    }


  cv::Mat dockOgridDraw = cv::Mat::zeros((int)(ogrid_size / resolution), (int)(ogrid_size / resolution), CV_8U);
  for(size_t i = 0; i < ping_msg->ranges.size(); ++i) {
    if(ping_msg->intensities.at(i) > 2000) { //TODO: Better thresholding
      
      //Get distance. RIGHT TRIANGLES
      double x_d = ping_msg->ranges.at(i) * cos(ping_msg->bearings.at(i));
      double y_d = ping_msg->ranges.at(i) * sin(ping_msg->bearings.at(i));
      
      //Transform point using TF
      tf::Vector3 vec = tf::Vector3(x_d, y_d, 0);
      tf::Vector3 newVec = transform.getBasis()*vec;

      //Where to draw in opencv mat
      int x = newVec.x()/resolution + transform.getOrigin().x()/resolution + dockOgridDraw.cols/2;
      int y = newVec.y()/resolution + transform.getOrigin().y()/resolution + dockOgridDraw.rows/2;

      //Check if point is inside the OGRID
      cv::Rect rect(cv::Point(), dockOgridDraw.size());
      cv::Point p(x, y);
      if (rect.contains(p) && dockOgridDraw.at<uchar>(y, x) == 0 && newVec.z() > -pool_depth)
      {
        dockOgridDraw.at<uchar>(y,x) = 99;
        pcl::PointXYZI point; 
        point.x = newVec.x() + transform.getOrigin().x();
        point.y = newVec.y() + transform.getOrigin().y();
        point.z = newVec.z() + transform.getOrigin().z();
        point.intensity = ping_msg->intensities.at(i);
        pointCloudBuffer.push_back(point);
      }
    }
  }


  //Flatten Ogrid
  nav_msgs::OccupancyGrid rosGrid;
  std::vector<int8_t> data;
  for(int i = 0; i < dockOgridDraw.cols; ++i) {
    for (int j = 0; j < dockOgridDraw.rows; j++) {
      data.push_back((int)dockOgridDraw.at<uint8_t>(cv::Point(i, j)));
    }
  }

  //Publish Ogrid
  rosGrid.header.seq = 0;
  rosGrid.info.resolution = resolution;
  rosGrid.header.frame_id = "map";
  rosGrid.header.stamp = ros::Time::now();
  rosGrid.info.map_load_time = ros::Time::now();
  rosGrid.info.width = dockOgridDraw.cols;
  rosGrid.info.height = dockOgridDraw.rows;
  rosGrid.info.origin.position.x = -ogrid_size/2;
  rosGrid.info.origin.position.y = -ogrid_size/2;
  rosGrid.info.origin.position.z = 0;
  rosGrid.data = data;
  pubGrid.publish(rosGrid);

  //Publish point cloud
  pcl::PointCloud<pcl::PointXYZI> pointCloud;
  for(auto &p : pointCloudBuffer) {
    pointCloud.push_back(p);
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pointCloud,output);
  output.header.frame_id = "map";
  output.header.stamp = ros::Time::now();
  pubPointCloud.publish(output);

}


void OGridGen::callback(const blueview_driver::BlueViewPingPtr &ping_msg)
{
  publish_ogrid(ping_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ogridgen");
  ros::NodeHandle nh;
  OGridGen oGridGen(&nh);
  ros::Subscriber sub = nh.subscribe("/blueview_driver/ranges", 1, &OGridGen::callback, &oGridGen);
  ros::spin();
}