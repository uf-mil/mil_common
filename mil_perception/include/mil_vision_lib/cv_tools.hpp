#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace mil_vision {
    /** Retrieves a paramater of a list of list of doubles and puts them into a vector of opencv 3D points
     * @param topic Ros topic in the global namespace to load points from
     * @param points A vector of cv::Point3f to put points in from the topic
     * @return True if points loaded successfully, False if an error occurred (details will be in ros error output)
     */
    bool get3DPointsFromParam(std::string topic, std::vector<cv::Point3f>& points);
    /// @param value A ROS XML value for parsing points from an already resolved param
    bool get3DPointsFromParam(XmlRpc::XmlRpcValue& value, std::vector<cv::Point3f>& points);
    /// @param nh Ros nodehandle to retrieve topic from
    /// @param topic Ros topic in nh's namespace to load points from
    bool get3DPointsFromParam(ros::NodeHandle& nh, std::string topic, std::vector<cv::Point3f>& points);

    /** Creates an image displaying a vector of 3D points
     *  Useful for visualizing models of objects used in 3D pose estimation with solvePnP
     * @param points A vector of 3D points, centered around (0, 0, 0)
     * @return An opencv image displaying the points with each labeled with it's index. Depth (z) is ignored
     */ 
    /// Useful for testing 3D point representations used for pose estimation (like with solvePnP)
    cv::Mat displayZeroCentered3DPoints(std::vector<cv::Point3f>& points);
}
