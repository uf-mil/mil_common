#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

namespace pcodar
{

class ObjectMap
{

public:
  ObjectMap();
  mil_msgs::PerceptionObjectArray to_msg();
  id_object_map objects_;
  size_t greatest_id;
};

} // namespace pcodar
