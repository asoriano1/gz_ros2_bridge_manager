#pragma once

#include <string>
#include <vector>

namespace gz_ros2_bridge_manager
{

struct SensorEntry
{
  std::string linkName;
  std::string sensorName;
  std::string sensorType;  // e.g. "camera", "lidar"
};

struct ModelSensorInfo
{
  std::string modelName;
  std::vector<SensorEntry> sensors;
  bool available = false;
  std::string note;
};

// Attempts to retrieve model → link → sensor hierarchy from Gazebo.
//
// Not yet implemented: the gz-sim scene/info service exposes models and
// visuals but not the sensor hierarchy (type, topic assignments) without
// parsing SDF source. This class is a placeholder; a future implementation
// may use the /world/<name>/state topic or a dedicated sensor service.
class ModelSensorDiscovery
{
public:
  // Returns false: sensor hierarchy discovery is not yet implemented.
  static bool isAvailable();

  // Returns a human-readable explanation of why the feature is unavailable.
  static std::string unavailableReason();

  // Placeholder — always returns an empty/unavailable ModelSensorInfo.
  ModelSensorInfo discoverForModel(const std::string &worldName,
                                   const std::string &modelName) const;
};

}  // namespace gz_ros2_bridge_manager
