#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

namespace gz_ros2_bridge_manager
{

// Generates `ros2 run ros_gz_bridge parameter_bridge` commands from
// candidate topics. Pure functions, no Qt or Gazebo dependencies.
class BridgeCommandBuilder
{
public:
  // Returns specs from candidates that are bridgeable AND checked,
  // deduplicated, preserving input order.
  static std::vector<std::string> selectedSpecs(
      const std::vector<BridgeTopicCandidate> &candidates);

  // Single-line command (suitable for clipboard).
  // Returns "" if no specs are checked.
  static std::string buildCommand(
      const std::vector<BridgeTopicCandidate> &candidates);

  // Multi-line, line-continuation form (suitable for display in a terminal).
  // Returns "" if no specs are checked.
  static std::string buildCommandWrapped(
      const std::vector<BridgeTopicCandidate> &candidates);
};

}  // namespace gz_ros2_bridge_manager
