#include "gz_ros2_bridge_manager/ModelSensorDiscovery.hh"

namespace gz_ros2_bridge_manager
{

bool ModelSensorDiscovery::isAvailable()
{
  return false;
}

std::string ModelSensorDiscovery::unavailableReason()
{
  return
    "Model → link → sensor hierarchy is not yet implemented. "
    "The /world/<name>/scene/info service exposes visuals but not sensor types "
    "or topic assignments. A future version may use the SDF introspection "
    "service or the /world/<name>/state topic.";
}

ModelSensorInfo ModelSensorDiscovery::discoverForModel(
    const std::string & /*worldName*/,
    const std::string &modelName) const
{
  ModelSensorInfo info;
  info.modelName  = modelName;
  info.available  = false;
  info.note       = unavailableReason();
  return info;
}

}  // namespace gz_ros2_bridge_manager
