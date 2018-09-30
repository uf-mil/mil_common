#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"
#include "object_map.hpp"

#include <mil_msgs/PerceptionObject.h>

#include <limits>

namespace pcodar
{

extern uint NO_ASSOCIATION_FOUND;

struct association_unit
{
    uint index;
    uint object_id;
};

class associator
{
   public:
    associator()
    {
    }
    void associate(ObjectMap& prev_objects, point_cloud const& pc, clusters_t clusters);
};

}  // namespace pcod#pragma once
