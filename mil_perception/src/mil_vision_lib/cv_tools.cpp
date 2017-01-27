#include <mil_vision_lib/cv_tools.hpp>

bool mil_vision::get3DPointsFromParam(std::string topic, std::vector<cv::Point3f>& points)
{
    XmlRpc::XmlRpcValue list;
    if (!ros::param::get(topic, list)) return false;
    return mil_vision::get3DPointsFromParam(list, points);
}
bool mil_vision::get3DPointsFromParam(ros::NodeHandle& nh, std::string topic, std::vector<cv::Point3f>& points)
{
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam(topic, list)) return false;
    return mil_vision::get3DPointsFromParam(list, points);   
}
bool mil_vision::get3DPointsFromParam(XmlRpc::XmlRpcValue& list, std::vector<cv::Point3f>& points)
{
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_DEBUG("Cannot parse param as points, not an array");
        return false;
    }
    for (int32_t o = 0; o < list.size(); o++)
    {
        if (list[o].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_DEBUG("Cannot parse param as points, not an array");
            return false;
        } else if (list[o].size() != 3) {
            ROS_DEBUG("Cannot parse param as points, not 3 values per list");
            return false;
        } else if (list[o][0].getType() != XmlRpc::XmlRpcValue::TypeDouble || 
                   list[o][1].getType() != XmlRpc::XmlRpcValue::TypeDouble || 
                   list[o][2].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
            ROS_DEBUG("Cannot parse param as points, values not doubles");
            return false; 
        }
        cv::Point3f p;
        p.x = static_cast<double>(list[o][0]);
        p.y = static_cast<double>(list[o][1]);
        p.z = static_cast<double>(list[o][2]);
        points.push_back(p);
    }
    return true;
}
cv::Mat mil_vision::displayZeroCentered3DPoints(std::vector<cv::Point3f>& points)
{
    const int offset = 300;
    const double scale = 500;
    double max_x = 0;
    double max_y = 0;
    for (size_t i = 0; i < points.size(); i ++)
    {
        if (std::abs(points[i].x) > max_x) max_x = std::abs(points[i].x);
        if (std::abs(points[i].y) > max_y) max_y = std::abs(points[i].y); 
    }
    max_x *= scale;
    max_y *= scale;
    cv::Mat mat = cv::Mat::zeros(max_x*2+offset, max_y*2+offset, CV_8UC3);
    for (size_t i = 0; i < points.size(); i++)
    {
        cv::Point p;
        p.x = points[i].x*scale + max_x + offset/2;
        p.y = points[i].y*scale + max_y + offset/2;
        putText(mat, std::to_string(i),
          cv::Point(p.x + 5, p.y), CV_FONT_HERSHEY_SIMPLEX, .6,
          cv::Scalar(0,0,255), 1, CV_AA);
        cv::circle(mat, p, 3, cv::Scalar(255,0,0), -1);
    }
    return mat;
}
