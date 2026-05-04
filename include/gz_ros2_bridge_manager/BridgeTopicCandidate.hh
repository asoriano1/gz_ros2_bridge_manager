#pragma once

#include <string>

namespace gz_ros2_bridge_manager
{

// Confidence categories for topic-to-model association.
enum class AssociationCategory
{
  Unsupported = 0,
  Additional = 1,
  EcmAssociated = 2,
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

  bool bridgeable = false;       // ros_gz_bridge supports the type
  bool checked = false;          // current effective check state (default + user override)
  std::string warning;           // optional explanation surfaced in UI
};

}  // namespace gz_ros2_bridge_manager
