#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"
#include "object.hpp"

#include <mil_msgs/PerceptionObjectArray.h>

namespace pcodar
{

class ObjectMap
{

public:
  ObjectMap();
  mil_msgs::PerceptionObjectArray to_msg();
  std::unordered_map<uint, Object> objects_;
  size_t greatest_id;
};

} // namespace pcodar
