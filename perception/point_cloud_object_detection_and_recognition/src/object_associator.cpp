#include <point_cloud_object_detection_and_recognition/object_associator.hpp>

#include <pcl/search/octree.h>
#include <mil_msgs/PerceptionObject.h>

namespace pcodar
{
uint NO_ASSOCIATION_FOUND = std::numeric_limits<uint>::max();

void associator::associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters)
{
    if (prev_objects.objects_.empty()) {
        for (cluster_t const& cluster : clusters) {
            point_cloud cluster_pc(pc, cluster.indices);
            ROS_INFO("NEW OBJECT");
            prev_objects.add_object(cluster_pc);
        }
    }

    pcl::search::Octree<point_t> search(0.01);
    for (cluster_t const& cluster : clusters)
    {
        point_cloud_ptr cluster_pc = boost::make_shared<point_cloud>(pc, cluster.indices);
        search.setInputCloud(cluster_pc);
        float min = std::numeric_limits<float>::max();
        std::unordered_map<uint, Object>::iterator it = prev_objects.objects_.end();
        for (auto pair = prev_objects.objects_.begin(); pair != prev_objects.objects_.end(); ++pair)
        {
            int index = 0;
            float distance = 0.;
            search.approxNearestSearch((*pair).second.center_, index, distance);
            ROS_INFO("DISTANCE IS %f MAX_DISTANCE=%f", distance, params.max_distance_for_association);
            if (distance < min && distance < params.max_distance_for_association)
            {
                min = distance;
                it = pair;
            }
        }
        ROS_INFO("MIN DISTANCE is %f", min);

        if (it == prev_objects.objects_.end()) {
            // Add to object
            ROS_INFO("NEW OBJECT");
            prev_objects.add_object(*cluster_pc);
        } else {
            ROS_INFO("UPDATING POINTS for %ud", (*it).first);
            (*it).second.update_points(*cluster_pc);
        }
    }
}

}  // namespace pcodar
