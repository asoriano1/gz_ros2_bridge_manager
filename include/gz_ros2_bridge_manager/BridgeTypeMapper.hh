#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace gz_ros2_bridge_manager
{

// Maps Gazebo message types to ROS 2 message types and generates bridge specs.
// All methods are thread-safe (const, no shared mutable state).
class BridgeTypeMapper
{
public:
  BridgeTypeMapper();

  // Returns true if the given gz message type has a known ROS 2 equivalent.
  bool isBridgeable(const std::string &gzMsgType) const;

  // Returns the ROS 2 message type for a gz message type, or "" if unknown.
  std::string ros2Type(const std::string &gzMsgType) const;

  // Returns the bridge spec string: /topic@ros_type@gz_type
  // Returns "" if the type is not bridgeable.
  std::string bridgeSpec(const std::string &topic,
                          const std::string &gzMsgType) const;

  // Returns all known gz message types.
  std::vector<std::string> knownGzTypes() const;

private:
  std::unordered_map<std::string, std::string> mappings_;
};

}  // namespace gz_ros2_bridge_manager
