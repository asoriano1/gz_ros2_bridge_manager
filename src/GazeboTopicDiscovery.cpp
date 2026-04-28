#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"

#include <vector>

#include <gz/transport/Node.hh>
#include <gz/common/Console.hh>

namespace gz_ros2_bridge_manager
{

std::vector<GzTopicEntry> GazeboTopicDiscovery::discover(
    const BridgeTypeMapper &mapper) const
{
  gz::transport::Node node;
  std::vector<std::string> topicNames;
  node.TopicList(topicNames);

  std::vector<GzTopicEntry> entries;
  entries.reserve(topicNames.size());

  for (const auto &topic : topicNames)
  {
    GzTopicEntry entry;
    entry.topicName = topic;
    entry.gzMsgType = getMessageType(topic);
    entry.bridgeable = mapper.isBridgeable(entry.gzMsgType);
    entry.ros2MsgType = mapper.ros2Type(entry.gzMsgType);
    entry.bridgeSpec = mapper.bridgeSpec(topic, entry.gzMsgType);
    entries.push_back(std::move(entry));
  }

  return entries;
}

std::string GazeboTopicDiscovery::getMessageType(const std::string &topic) const
{
  gz::transport::Node node;
  std::vector<gz::transport::MessagePublisher> publishers;
  std::vector<gz::transport::MessagePublisher> subscribers;

  if (!node.TopicInfo(topic, publishers, subscribers) || publishers.empty())
    return {};

  // A topic can have multiple publishers with different types (rare in practice).
  // Return the type from the first publisher; all publishers on a gz topic
  // are expected to use the same message type.
  return publishers.front().MsgTypeName();
}

}  // namespace gz_ros2_bridge_manager
