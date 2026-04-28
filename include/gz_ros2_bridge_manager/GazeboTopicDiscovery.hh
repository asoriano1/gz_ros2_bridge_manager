#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"

namespace gz_ros2_bridge_manager
{

struct GzTopicEntry
{
  std::string topicName;
  std::string gzMsgType;   // e.g. "gz.msgs.LaserScan"
  bool bridgeable = false;
  std::string ros2MsgType; // e.g. "sensor_msgs/msg/LaserScan"
  std::string bridgeSpec;  // e.g. "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
};

// Discovers Gazebo Transport topics and classifies them for bridging.
// All methods run synchronously; call from a background thread.
class GazeboTopicDiscovery
{
public:
  // Returns all topics currently visible on gz-transport, annotated with
  // message type information and bridge specs where available.
  std::vector<GzTopicEntry> discover(const BridgeTypeMapper &mapper) const;

private:
  std::string getMessageType(const std::string &topic) const;
};

}  // namespace gz_ros2_bridge_manager
