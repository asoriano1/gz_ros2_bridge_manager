#pragma once

#include <string>

namespace gz_ros2_bridge_manager
{

// Confidence categories for topic-to-model association.
// Ordering reflects strength: higher value = stronger association.
enum class AssociationCategory
{
  Unsupported = 0,              // not bridgeable at all
  CompatibleButUnassigned = 1,  // bridgeable, no confident model match
  ContainsModelName = 2,        // model name appears as a token in topic path
  ContainsSanitizedModelName = 3, // sanitized form appears as a token
  ExactModelPath = 4,           // /model/<name>/ or /world/<w>/model/<name>/
};

const char *categoryName(AssociationCategory c);

// Single bridgeable-or-not topic candidate, fully self-described for QML/UI.
struct BridgeTopicCandidate
{
  std::string gzTopic;
  std::string gzType;            // e.g. "gz.msgs.LaserScan"
  std::string ros2Type;          // e.g. "sensor_msgs/msg/LaserScan"
  std::string bridgeSpec;        // e.g. "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"

  AssociationCategory category = AssociationCategory::Unsupported;
  std::string confidenceLabel;   // human-readable: "Strong (model path)", "Unassigned", ...

  bool bridgeable = false;       // ros_gz_bridge supports the type
  bool checked = false;          // current effective check state (default + user override)
  bool ambiguous = false;        // matched the selected model AND another model
  bool isGeneric = false;        // generic ROS-style topic (/clock, /scan, /tf, ...)
  std::string warning;           // optional explanation surfaced in UI
};

}  // namespace gz_ros2_bridge_manager
