#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace gz_ros2_bridge_manager
{

/// Maps Gazebo message types onto their ROS 2 equivalents and generates
/// parameter_bridge spec strings.
///
/// The mapping table is fixed at construction time so all queries are const
/// and therefore safe to call from any thread.
class BridgeTypeMapper
{
public:
  BridgeTypeMapper();

  /// Returns true iff `gzMsgType` has a known ROS 2 equivalent.
  bool isBridgeable(const std::string &gzMsgType) const;

  /// Returns the ROS 2 message type associated with `gzMsgType`, or "" when
  /// the type is unknown.
  std::string ros2Type(const std::string &gzMsgType) const;

  /// Builds the parameter_bridge spec `/topic@ros_type@gz_type`.
  /// Returns "" when the gz type is not bridgeable.
  std::string bridgeSpec(const std::string &topic,
                          const std::string &gzMsgType) const;

  /// Returns the full list of Gazebo message types known to the mapper.
  std::vector<std::string> knownGzTypes() const;

private:
  std::unordered_map<std::string, std::string> mappings_;
};

}  // namespace gz_ros2_bridge_manager
