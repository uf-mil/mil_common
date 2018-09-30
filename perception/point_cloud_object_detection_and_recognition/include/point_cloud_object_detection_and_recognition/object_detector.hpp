#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcodar
{
class object_detector
{
   public:
    object_detector() {};
    clusters_t get_clusters(point_cloud_const_ptr pc);
};
}
