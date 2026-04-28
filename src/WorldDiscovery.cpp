#include "gz_ros2_bridge_manager/WorldDiscovery.hh"

#include <regex>
#include <vector>

#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/common/Console.hh>

namespace gz_ros2_bridge_manager
{

WorldInfo WorldDiscovery::discover() const
{
  WorldInfo info;
  info.worldName = discoverWorldName();

  if (info.worldName.empty())
  {
    info.errorMessage =
        "No active Gazebo world found. "
        "Is gz sim running and connected via gz-transport?";
    return info;
  }

  info.modelNames = queryModels(info.worldName);
  return info;
}

std::string WorldDiscovery::discoverWorldName() const
{
  // Pattern: /world/<name>/stats is always published by an active gz-sim world.
  gz::transport::Node node;
  std::vector<std::string> topics;
  node.TopicList(topics);

  const std::regex statsPattern(R"(^/world/([^/]+)/stats$)");
  std::smatch m;
  for (const auto &t : topics)
  {
    if (std::regex_match(t, m, statsPattern))
      return m[1].str();
  }
  return {};
}

std::vector<std::string> WorldDiscovery::queryModels(
    const std::string &worldName) const
{
  const std::string service = "/world/" + worldName + "/scene/info";

  gz::transport::Node node;
  gz::msgs::Empty req;
  gz::msgs::Scene rep;
  bool result = false;

  const bool called = node.Request(service, req, kServiceTimeoutMs, rep, result);
  if (!called || !result)
  {
    gzwarn << "[gz_ros2_bridge_manager] queryModels: scene/info call failed "
           << "for world '" << worldName << "'\n";
    return {};
  }

  std::vector<std::string> names;
  names.reserve(static_cast<size_t>(rep.model_size()));
  for (int i = 0; i < rep.model_size(); ++i)
    names.push_back(rep.model(i).name());
  return names;
}

}  // namespace gz_ros2_bridge_manager
