#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace pcodar
{
class object_detector
{
   public:
    object_detector() {};
    mil_msgs::PerceptionObjectArrayPtr get_objects(const point_cloud& pc);
};
}
