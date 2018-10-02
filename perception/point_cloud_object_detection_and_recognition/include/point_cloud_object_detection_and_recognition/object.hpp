#pragma once

#include "pcodar_params.hpp"
#include "pcodar_types.hpp"

namespace pcodar
{
class Object
{
public:
  Object(point_cloud const& pc);
  void update_points(point_cloud const& pc);
  mil_msgs::PerceptionObject to_msg();
  point_cloud points_;
private:
};

} // namespace pcodar
