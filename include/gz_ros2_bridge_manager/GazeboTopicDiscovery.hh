#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"

namespace gz_ros2_bridge_manager
{

/// One topic discovered on gz-transport, already annotated with everything
/// needed to decide whether and how to bridge it.
struct GzTopicEntry
{
  std::string topicName;     ///< Gazebo Transport topic name, e.g. "/scan".
  std::string gzMsgType;     ///< Gazebo message type, e.g. "gz.msgs.LaserScan".
  bool bridgeable = false;   ///< True when a ROS 2 equivalent exists.
  std::string ros2MsgType;   ///< ROS 2 type, e.g. "sensor_msgs/msg/LaserScan".
  std::string bridgeSpec;    ///< Full parameter_bridge spec, when bridgeable.
};

/// Discovers Gazebo Transport topics and classifies each one for bridging.
///
/// The discovery call is synchronous (it queries `gz topic -l` style data
/// from the transport node), so callers must invoke it from a background
/// thread to keep the GUI responsive.
class GazeboTopicDiscovery
{
public:
  /// Returns every topic currently advertised on gz-transport, annotated
  /// with its message type and bridge spec when known by `mapper`.
  std::vector<GzTopicEntry> discover(const BridgeTypeMapper &mapper) const;

private:
  std::string getMessageType(const std::string &topic) const;
};

}  // namespace gz_ros2_bridge_manager
