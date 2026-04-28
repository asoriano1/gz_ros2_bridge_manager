#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"

namespace gz_ros2_bridge_manager
{

struct AssociationResult
{
  std::vector<BridgeTopicCandidate> associated;   // confidently linked to selected model
  std::vector<BridgeTopicCandidate> unassigned;   // bridgeable but not associated
  std::vector<BridgeTopicCandidate> unsupported;  // not bridgeable
  std::vector<std::string> warnings;
};

// Classifies a list of discovered Gazebo topics with respect to a selected
// model, producing a transparent confidence label per topic.
//
// Heuristic in order of priority (highest wins):
//   1. ExactModelPath          — topic contains /model/<name>/ or
//                                /world/<world>/model/<name>/
//   2. ContainsSanitizedModelName — sanitized model name as a token in path
//   3. ContainsModelName       — original model name as a token in path
//   4. CompatibleButUnassigned — bridgeable but doesn't match selected model
//   5. Unsupported             — not bridgeable
//
// Generic ROS-style topics (/clock, /scan, /tf, ...) are flagged separately
// and never auto-associated to a model unless the path explicitly references
// the model.
class TopicAssociationHeuristic
{
public:
  // Classify topics with respect to selectedModel within worldName.
  // allModels is used to detect ambiguity (when a longer model name also
  // matches the same topic).
  // If selectedModel is empty: all bridgeable topics go to "unassigned",
  // none are checked by default.
  AssociationResult associate(
      const std::string &selectedModel,
      const std::string &worldName,
      const std::vector<GzTopicEntry> &topics,
      const std::vector<std::string> &allModels) const;

  // ---- Helpers exposed for testing ----

  // Lowercase, replace non-alphanumerics with '_', collapse repeats, trim.
  static std::string sanitizeName(const std::string &name);

  // Returns true if topic matches a well-known generic/global topic pattern.
  static bool isGenericTopic(const std::string &topic);

  // Substring match where left/right boundaries are non-identifier characters
  // ([^a-zA-Z0-9_]). Empty needle returns false.
  static bool tokenContains(const std::string &haystack,
                            const std::string &needle);

  // Returns true if topic contains the strong scoped path:
  //   /model/<name>/   OR   /world/<world>/model/<name>/
  static bool hasExactModelPath(const std::string &topic,
                                const std::string &worldName,
                                const std::string &modelName);
};

}  // namespace gz_ros2_bridge_manager
