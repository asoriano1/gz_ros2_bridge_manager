#include "gz_ros2_bridge_manager/BridgeCommandBuilder.hh"

#include <unordered_set>

namespace gz_ros2_bridge_manager
{

std::vector<std::string> BridgeCommandBuilder::selectedSpecs(
    const std::vector<BridgeTopicCandidate> &candidates)
{
  std::vector<std::string> specs;
  std::unordered_set<std::string> seen;
  specs.reserve(candidates.size());

  for (const auto &c : candidates)
  {
    if (!c.bridgeable || !c.checked || c.bridgeSpec.empty())
      continue;
    if (seen.insert(c.bridgeSpec).second)
      specs.push_back(c.bridgeSpec);
  }
  return specs;
}

std::string BridgeCommandBuilder::buildCommand(
    const std::vector<BridgeTopicCandidate> &candidates)
{
  const auto specs = selectedSpecs(candidates);
  if (specs.empty())
    return {};

  std::string cmd = "ros2 run ros_gz_bridge parameter_bridge";
  for (const auto &s : specs)
  {
    cmd.push_back(' ');
    cmd.append(s);
  }
  return cmd;
}

std::string BridgeCommandBuilder::buildCommandWrapped(
    const std::vector<BridgeTopicCandidate> &candidates)
{
  const auto specs = selectedSpecs(candidates);
  if (specs.empty())
    return {};

  std::string cmd = "ros2 run ros_gz_bridge parameter_bridge";
  for (const auto &s : specs)
  {
    cmd.append(" \\\n  ");
    cmd.append(s);
  }
  return cmd;
}

}  // namespace gz_ros2_bridge_manager
