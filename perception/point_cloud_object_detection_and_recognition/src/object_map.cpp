#include <point_cloud_object_detection_and_recognition/object_map.hpp>
#include <mil_msgs/PerceptionObject.h>

namespace pcodar
{

ObjectMap::ObjectMap()
{
}

mil_msgs::PerceptionObjectArray ObjectMap::to_msg()
{
  mil_msgs::PerceptionObjectArray msg;
  for (auto& pair : objects_)
  {
     mil_msgs::PerceptionObject object_msg = pair.second.to_msg();
     object_msg.id = pair.first;
     msg.objects.push_back(object_msg);
  }
  return msg;
}

} // namespace pcodar
