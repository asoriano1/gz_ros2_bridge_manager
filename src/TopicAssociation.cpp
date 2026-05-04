#include "gz_ros2_bridge_manager/TopicAssociation.hh"

#include <algorithm>
#include <cctype>
#include <unordered_set>

namespace gz_ros2_bridge_manager
{

namespace
{

struct InferredBridgeType
{
  std::string gzType;
  std::string ros2Type;
};

std::string normalizeTopicValue(const std::string &topic)
{
  return TopicAssociation::normalizeTopic(topic);
}

const GzTopicEntry *findExactTopicEntry(
    const std::string &topic,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  const auto normalized = normalizeTopicValue(topic);
  for (const auto &entry : advertisedTopics)
  {
    if (normalizeTopicValue(entry.topicName) == normalized)
      return &entry;
  }
  return nullptr;
}

std::string topicDirectory(const std::string &topic)
{
  const auto normalized = normalizeTopicValue(topic);
  const auto pos = normalized.find_last_of('/');
  if (pos == std::string::npos || pos == 0u)
    return {};
  return normalized.substr(0, pos);
}

std::string topicLeaf(const std::string &topic)
{
  const auto normalized = normalizeTopicValue(topic);
  const auto pos = normalized.find_last_of('/');
  if (pos == std::string::npos)
    return normalized;
  if (pos + 1u >= normalized.size())
    return {};
  return normalized.substr(pos + 1u);
}

std::string gzTypeFromBridgeSpec(const std::string &spec)
{
  const auto at1 = spec.find('@');
  if (at1 == std::string::npos)
    return {};
  const auto at2 = spec.find('@', at1 + 1);
  if (at2 == std::string::npos || at2 + 1 >= spec.size())
    return {};
  return spec.substr(at2 + 1);
}

std::string normalizeBridgeSpecTopic(const std::string &spec,
                                     const std::string &normalizedTopic)
{
  const auto at1 = spec.find('@');
  if (at1 == std::string::npos)
    return spec;
  return normalizedTopic + spec.substr(at1);
}

InferredBridgeType inferBridgeTypeForSensor(const std::string &sensorType)
{
  if (sensorType == "camera")
    return {"gz.msgs.Image", "sensor_msgs/msg/Image"};
  if (sensorType == "imu")
    return {"gz.msgs.IMU", "sensor_msgs/msg/Imu"};
  if (sensorType == "gpu_lidar" || sensorType == "lidar" || sensorType == "ray")
    return {"gz.msgs.LaserScan", "sensor_msgs/msg/LaserScan"};
  if (sensorType == "navsat" || sensorType == "gps")
    return {"gz.msgs.NavSat", "sensor_msgs/msg/NavSatFix"};
  return {};
}

bool isCameraLikeSensor(const std::string &sensorType)
{
  return sensorType == "camera" ||
         sensorType == "depth_camera" ||
         sensorType == "rgbd_camera";
}

bool isAdvertisedCameraInfoTopic(const GzTopicEntry &entry)
{
  return !entry.topicName.empty() &&
         entry.gzMsgType == "gz.msgs.CameraInfo" &&
         entry.bridgeable &&
         !entry.bridgeSpec.empty();
}

std::vector<GzTopicEntry> findAdvertisedCameraInfoTopics(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  std::vector<GzTopicEntry> matches;
  if (!isCameraLikeSensor(sensor.sensorType) || sensor.declaredTopic.empty())
    return matches;

  const auto normalizedTopic = normalizeTopicValue(sensor.declaredTopic);
  const auto topicDir = topicDirectory(normalizedTopic);
  if (topicDir.empty())
    return matches;

  const std::string directSibling = topicDir + "/camera_info";
  const std::string directSiblingRaw = topicDir + "/camera_info_raw";
  std::unordered_set<std::string> seenTopics;

  for (const auto &entry : advertisedTopics)
  {
    if (!isAdvertisedCameraInfoTopic(entry))
      continue;

    const auto normalizedEntry = normalizeTopicValue(entry.topicName);
    const auto entryLeaf = topicLeaf(normalizedEntry);
    const bool exactSibling =
        normalizedEntry == directSibling ||
        normalizedEntry == directSiblingRaw;
    const bool sameNamespace =
        normalizedEntry.rfind(topicDir + "/", 0) == 0 &&
        (entryLeaf == "camera_info" || entryLeaf == "camera_info_raw");

    if (!exactSibling && !sameNamespace)
      continue;

    if (!seenTopics.insert(normalizedEntry).second)
      continue;
    matches.push_back(entry);
  }

  return matches;
}

void appendTopicMatch(
    DiscoveredSensor &sensor,
    const GzTopicEntry &topic,
    std::unordered_set<std::string> &seenSpecs,
    std::unordered_set<std::string> *claimedTopics)
{
  if (!seenSpecs.insert(topic.bridgeSpec).second)
    return;

  const std::string normalizedTopic = normalizeTopicValue(topic.topicName);
  sensor.matchedTopicNames.push_back(normalizedTopic);
  sensor.matchedBridgeSpecs.push_back(
      normalizeBridgeSpecTopic(topic.bridgeSpec, normalizedTopic));
  if (claimedTopics != nullptr)
    claimedTopics->insert(normalizedTopic);
}

}  // namespace

const char *matchSourceName(MatchSource s)
{
  switch (s)
  {
    case MatchSource::EcmSensorTopicExact:   return "ECM exact";
    case MatchSource::EcmSensorTopicPrefix:  return "ECM prefix";
    case MatchSource::Unresolved:            return "Unresolved";
    default:                                 return "Unknown";
  }
}

std::vector<std::string> TopicAssociation::suffixesForType(const std::string &t)
{
  if (t == "camera")
    return {"/image", "/camera_info"};
  if (t == "depth_camera")
    return {"/image", "/camera_info", "/depth", "/depth_raw", "/points"};
  if (t == "rgbd_camera")
    return {"/image", "/camera_info", "/depth", "/depth_raw", "/points"};
  if (t == "gpu_lidar" || t == "lidar")
    return {"/scan", "/points"};
  if (t == "imu")
    return {"/imu"};
  if (t == "force_torque")
    return {"/wrench"};
  if (t == "magnetometer")
    return {"/magnetometer"};
  if (t == "air_pressure")
    return {"/air_pressure"};
  if (t == "navsat" || t == "gps")
    return {"/navsat"};
  return {""};
}

std::string TopicAssociation::normalizeTopic(const std::string &topic)
{
  if (topic.empty())
    return {};

  size_t start = 0;
  while (start < topic.size() &&
         std::isspace(static_cast<unsigned char>(topic[start])) != 0)
  {
    ++start;
  }

  size_t end = topic.size();
  while (end > start &&
         std::isspace(static_cast<unsigned char>(topic[end - 1])) != 0)
  {
    --end;
  }

  if (start >= end)
    return {};

  std::string normalized = topic.substr(start, end - start);
  if (!normalized.empty() && normalized.front() != '/')
    normalized.insert(normalized.begin(), '/');
  while (normalized.size() > 1u && normalized.back() == '/')
    normalized.pop_back();
  return normalized;
}

std::unordered_set<std::string> TopicAssociation::claimedTopicNames(
    const ModelSensorTree &tree)
{
  std::unordered_set<std::string> claimed;
  for (const auto &sensor : tree.sensors)
  {
    for (const auto &topic : sensor.matchedTopicNames)
      claimed.insert(TopicAssociation::normalizeTopic(topic));
  }
  return claimed;
}

std::vector<GzTopicEntry> TopicAssociation::bridgeableTopicsExcludingClaims(
    const std::vector<GzTopicEntry> &discoveredTopics,
    const ModelSensorTree &tree)
{
  const auto claimed = claimedTopicNames(tree);
  std::vector<GzTopicEntry> result;
  for (const auto &entry : discoveredTopics)
  {
    if (!entry.bridgeable || entry.bridgeSpec.empty())
      continue;
    if (claimed.count(normalizeTopic(entry.topicName)) > 0)
      continue;
    result.push_back(entry);
  }
  return result;
}

MatchSource TopicAssociation::tryMatchPrefix(
    const std::string &prefix,
    const std::string &sensorType,
    const std::vector<GzTopicEntry> &advertisedTopics,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  if (prefix.empty())
    return MatchSource::Unresolved;

  const std::string normalizedPrefix = normalizeTopicValue(prefix);
  std::unordered_set<std::string> seenSpecs;
  bool exactPrefixMatch = false;
  bool suffixMatch = false;

  auto collect = [&](const GzTopicEntry &topic, bool isExact) -> bool
  {
    if (!topic.bridgeable || topic.bridgeSpec.empty())
      return false;
    if (!seenSpecs.insert(topic.bridgeSpec).second)
      return false;
    outTopicNames.push_back(normalizeTopicValue(topic.topicName));
    outBridgeSpecs.push_back(
        normalizeBridgeSpecTopic(topic.bridgeSpec, normalizeTopicValue(topic.topicName)));
    if (isExact)
      exactPrefixMatch = true;
    else
      suffixMatch = true;
    return true;
  };

  for (const auto &topic : advertisedTopics)
  {
    const std::string normalizedTopic = normalizeTopicValue(topic.topicName);
    if (normalizedTopic == normalizedPrefix)
    {
      collect(topic, true);
      continue;
    }

    if (normalizedTopic.size() > normalizedPrefix.size() &&
        normalizedTopic.compare(0, normalizedPrefix.size(), normalizedPrefix) == 0 &&
        normalizedTopic[normalizedPrefix.size()] == '/')
    {
      collect(topic, false);
    }
  }

  for (const auto &suffix : suffixesForType(sensorType))
  {
    if (suffix.empty())
      continue;

    const std::string candidate = normalizedPrefix + suffix;
    for (const auto &topic : advertisedTopics)
    {
      if (normalizeTopicValue(topic.topicName) == candidate)
        collect(topic, false);
    }
  }

  if (!exactPrefixMatch && !suffixMatch)
    return MatchSource::Unresolved;
  return exactPrefixMatch ? MatchSource::EcmSensorTopicExact
                          : MatchSource::EcmSensorTopicPrefix;
}

DiscoveredSensor TopicAssociation::matchSensor(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  DiscoveredSensor result;
  result.sensor = sensor;

  if (sensor.declaredTopic.empty())
  {
    result.warning = "no Sensor Topic in ECM";
    return result;
  }

  const GzTopicEntry *exactTopic =
      findExactTopicEntry(sensor.declaredTopic, advertisedTopics);
  result.topicListed = (exactTopic != nullptr);
  if (exactTopic != nullptr)
    result.topicInfoGzType = exactTopic->gzMsgType;

  auto matchSource = tryMatchPrefix(sensor.declaredTopic, sensor.sensorType,
                                    advertisedTopics,
                                    result.matchedTopicNames,
                                    result.matchedBridgeSpecs);
  if (matchSource != MatchSource::Unresolved)
  {
    result.matchSource = matchSource;
    result.resolved = true;
    result.typeSource = "advertised";
    if (result.topicInfoGzType.empty() && !result.matchedBridgeSpecs.empty())
      result.topicInfoGzType = gzTypeFromBridgeSpec(result.matchedBridgeSpecs.front());

    std::unordered_set<std::string> seenSpecs(result.matchedBridgeSpecs.begin(),
                                              result.matchedBridgeSpecs.end());
    for (const auto &cameraInfo : findAdvertisedCameraInfoTopics(sensor, advertisedTopics))
      appendTopicMatch(result, cameraInfo, seenSpecs, nullptr);
    return result;
  }

  if (exactTopic != nullptr && exactTopic->gzMsgType.empty())
  {
    const auto inferred = inferBridgeTypeForSensor(sensor.sensorType);
    if (!inferred.gzType.empty() && !inferred.ros2Type.empty())
    {
      const std::string normalizedDeclaredTopic =
          normalizeTopicValue(sensor.declaredTopic);
      result.matchedTopicNames.push_back(normalizedDeclaredTopic);
      result.matchedBridgeSpecs.push_back(
          normalizedDeclaredTopic + "@" + inferred.ros2Type + "@" + inferred.gzType);
      result.resolved = true;
      result.matchSource = MatchSource::EcmSensorTopicExact;
      result.inferredGzType = inferred.gzType;
      result.typeSource = "type inferred";
      result.warning =
          "Topic type inferred from ECM sensor type; TopicInfo not available yet.";

      std::unordered_set<std::string> seenSpecs(result.matchedBridgeSpecs.begin(),
                                                result.matchedBridgeSpecs.end());
      for (const auto &cameraInfo : findAdvertisedCameraInfoTopics(sensor, advertisedTopics))
        appendTopicMatch(result, cameraInfo, seenSpecs, nullptr);
      return result;
    }

    result.warning = "Sensor Topic listed but type cannot be inferred yet.";
    return result;
  }

  if (exactTopic != nullptr && !exactTopic->gzMsgType.empty())
  {
    result.warning = "Sensor Topic type is not bridgeable with ros_gz_bridge.";
    return result;
  }

  result.warning = "Sensor Topic not advertised in Gazebo Transport yet.";
  return result;
}

ModelSensorTree TopicAssociation::matchAll(
    const std::string &worldName,
    const std::vector<EcmSensorEntry> &sensors,
    const std::string &filterModelName,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  ModelSensorTree result;
  result.worldName = worldName;
  result.modelName = filterModelName;
  result.ecmConfirmed = true;

  std::unordered_set<std::string> claimedTopics;

  for (const auto &sensor : sensors)
  {
    DiscoveredSensor sensorResult = matchSensor(sensor, advertisedTopics);

    if (sensorResult.resolved)
    {
      const auto matchedTopics = sensorResult.matchedTopicNames;
      const auto matchedSpecs = sensorResult.matchedBridgeSpecs;
      sensorResult.matchedTopicNames.clear();
      sensorResult.matchedBridgeSpecs.clear();

      std::unordered_set<std::string> seenSpecs;
      for (size_t i = 0; i < matchedTopics.size() && i < matchedSpecs.size(); ++i)
      {
        const std::string normalizedTopic = normalizeTopicValue(matchedTopics[i]);
        if (claimedTopics.count(normalizedTopic) > 0)
          continue;

        GzTopicEntry topic;
        topic.topicName = normalizedTopic;
        topic.bridgeSpec = matchedSpecs[i];
        appendTopicMatch(sensorResult, topic, seenSpecs, &claimedTopics);
      }

      if (sensorResult.matchedTopicNames.empty())
      {
        sensorResult.resolved = false;
        sensorResult.warning =
            "Matching topic was already claimed by another model.";
      }
    }

    if (!sensorResult.resolved && sensorResult.warning.empty())
    {
      sensorResult.warning = "Sensor Topic not advertised in Gazebo Transport yet.";
    }

    if (!filterModelName.empty() && sensorResult.sensor.modelName != filterModelName)
      continue;
    result.sensors.push_back(std::move(sensorResult));
  }

  return result;
}

}  // namespace gz_ros2_bridge_manager
