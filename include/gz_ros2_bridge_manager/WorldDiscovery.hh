#pragma once

#include <string>
#include <vector>

namespace gz_ros2_bridge_manager
{

struct WorldInfo
{
  std::string worldName;
  std::vector<std::string> modelNames;
  std::string errorMessage;  // non-empty if discovery failed
};

// Discovers the active Gazebo world name and the models present in it.
// All methods run synchronously; call from a background thread.
class WorldDiscovery
{
public:
  // Timeout in milliseconds for the scene/info service call.
  static constexpr unsigned int kServiceTimeoutMs = 1500u;

  // Discovers the active world and its models.
  // World name is found by searching for the /world/<name>/stats topic.
  // Model list is obtained from the /world/<name>/scene/info service.
  WorldInfo discover() const;

private:
  std::string discoverWorldName() const;
  std::vector<std::string> queryModels(const std::string &worldName) const;
};

}  // namespace gz_ros2_bridge_manager
