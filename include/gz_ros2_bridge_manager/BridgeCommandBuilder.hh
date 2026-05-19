#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

namespace gz_ros2_bridge_manager
{

/// Generates `ros2 run ros_gz_bridge parameter_bridge` invocations from
/// a list of candidate topics.
///
/// All methods are pure free functions wrapped as static members — no Qt and
/// no Gazebo dependencies, so the builder is trivially unit-testable.
class BridgeCommandBuilder
{
public:
  /// Filters the input list down to the bridge specs that should actually be
  /// passed to the parameter_bridge.
  ///
  /// A candidate contributes its spec iff it is bridgeable and checked. The
  /// result is deduplicated while preserving input order so the displayed
  /// command line stays stable across refreshes.
  static std::vector<std::string> selectedSpecs(
      const std::vector<BridgeTopicCandidate> &candidates);

  /// Builds a single-line shell command, suitable for copying to the
  /// clipboard. Returns "" when no candidates are selected.
  static std::string buildCommand(
      const std::vector<BridgeTopicCandidate> &candidates);

  /// Builds the same command in multi-line form (backslash line-continuations),
  /// suitable for displaying in a terminal pane. Returns "" when no candidates
  /// are selected.
  static std::string buildCommandWrapped(
      const std::vector<BridgeTopicCandidate> &candidates);
};

}  // namespace gz_ros2_bridge_manager
