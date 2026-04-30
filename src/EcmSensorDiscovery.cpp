#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace gz_ros2_bridge_manager
{

// ---------------------------------------------------------------------------
// matchSourceName — human-readable label shown in the UI
// ---------------------------------------------------------------------------

const char *matchSourceName(MatchSource s)
{
  switch (s)
  {
    case MatchSource::EcmSensorTopicExact:     return "ECM exact";
    case MatchSource::EcmSensorTopicPrefix:    return "ECM prefix";
    case MatchSource::EcmStandardPrefix:       return "ECM path";
    case MatchSource::NameMatch:               return "Name match";
    case MatchSource::TypeCompatibleFallback:  return "Type fallback";
    case MatchSource::Unresolved:              return "Unresolved";
    default:                                   return "Unknown";
  }
}

// ---------------------------------------------------------------------------
// Suffix / ROS 2 type tables
// ---------------------------------------------------------------------------

std::vector<std::string> EcmTopicMatcher::suffixesForType(const std::string &t)
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

// ---------------------------------------------------------------------------
// defaultTopicPrefix — utility; worldName must be pre-baked for production use
// ---------------------------------------------------------------------------

std::string EcmTopicMatcher::defaultTopicPrefix(const std::string &worldName,
                                                  const EcmSensorEntry &s)
{
  if (worldName.empty() || s.modelName.empty() ||
      s.linkName.empty()  || s.sensorName.empty())
    return {};
  return "/world/" + worldName +
         "/model/" + s.modelName +
         "/link/"  + s.linkName  +
         "/sensor/" + s.sensorName;
}

std::string EcmTopicMatcher::normalizeTopic(const std::string &topic)
{
  if (topic.empty())
    return {};

  std::string normalized = topic;
  while (normalized.size() > 1u && normalized.back() == '/')
    normalized.pop_back();
  return normalized;
}

std::unordered_set<std::string> EcmTopicMatcher::claimedTopicNames(
    const ModelSensorTree &tree)
{
  std::unordered_set<std::string> claimed;
  for (const auto &sensor : tree.sensors)
  {
    if (!sensor.sensor.declaredTopic.empty())
      claimed.insert(EcmTopicMatcher::normalizeTopic(sensor.sensor.declaredTopic));
    for (const auto &topic : sensor.matchedTopicNames)
      claimed.insert(EcmTopicMatcher::normalizeTopic(topic));
  }
  return claimed;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

namespace
{

struct InferredBridgeType
{
  std::string gzType;
  std::string ros2Type;
};

std::string normalizeTopicValue(const std::string &topic)
{
  if (topic.empty())
    return {};

  std::string normalized = topic;
  while (normalized.size() > 1u && normalized.back() == '/')
    normalized.pop_back();
  return normalized;
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

// Returns true when `name` appears as an exact path segment in `topic`.
// E.g. "lidar" in "/model/robot/lidar/scan" → true
//      "lidar" in "/model/robotic/scan"      → false
bool isPathToken(const std::string &topic, const std::string &name)
{
  if (name.empty()) return false;
  size_t pos = 0;
  while (pos < topic.size())
  {
    const size_t sep = topic.find('/', pos);
    const size_t end = (sep == std::string::npos) ? topic.size() : sep;
    if (end > pos && topic.compare(pos, end - pos, name) == 0)
      return true;
    if (sep == std::string::npos) break;
    pos = sep + 1;
  }
  return false;
}

bool containsModelPath(const std::string &topic, const std::string &modelName)
{
  if (topic.empty() || modelName.empty())
    return false;

  const std::string needle = "/model/" + modelName;
  const auto pos = topic.find(needle);
  if (pos == std::string::npos)
    return false;

  const size_t end = pos + needle.size();
  return end == topic.size() || topic[end] == '/';
}

bool containsDifferentModelPath(const std::string &topic,
                                const std::string &modelName)
{
  const std::string marker = "/model/";
  const auto pos = topic.find(marker);
  if (pos == std::string::npos)
    return false;

  return !containsModelPath(topic, modelName);
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
  return entry.topicName.size() > 0u &&
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
  std::unordered_set<std::string> seenTopics;

  for (const auto &entry : advertisedTopics)
  {
    if (!isAdvertisedCameraInfoTopic(entry))
      continue;

    const auto normalizedEntry = normalizeTopicValue(entry.topicName);
    const auto entryLeaf = topicLeaf(normalizedEntry);
    const bool exactSibling = normalizedEntry == directSibling;
    const bool sameNamespace =
        normalizedEntry.rfind(topicDir + "/", 0) == 0 &&
        entryLeaf == "camera_info";

    if (!exactSibling && !sameNamespace)
      continue;

    if (!seenTopics.insert(normalizedEntry).second)
      continue;
    matches.push_back(entry);
  }

  return matches;
}

// Returns the expected ROS 2 message types for a sensor type.
std::vector<std::string> ros2TypesForSensor(const std::string &sensorType)
{
  if (sensorType == "camera")
    return {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo"};
  if (sensorType == "depth_camera")
    return {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
            "sensor_msgs/msg/PointCloud2"};
  if (sensorType == "rgbd_camera")
    return {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
            "sensor_msgs/msg/PointCloud2"};
  if (sensorType == "gpu_lidar" || sensorType == "lidar")
    return {"sensor_msgs/msg/LaserScan", "sensor_msgs/msg/PointCloud2"};
  if (sensorType == "imu")
    return {"sensor_msgs/msg/Imu"};
  if (sensorType == "force_torque")
    return {"geometry_msgs/msg/Wrench", "geometry_msgs/msg/WrenchStamped"};
  if (sensorType == "magnetometer")
    return {"sensor_msgs/msg/MagneticField"};
  if (sensorType == "air_pressure")
    return {"sensor_msgs/msg/FluidPressure"};
  if (sensorType == "navsat" || sensorType == "gps")
    return {"sensor_msgs/msg/NavSatFix"};
  return {};
}

bool ros2TypeCompatible(const EcmSensorEntry &sensor, const GzTopicEntry &topic)
{
  const auto expectedTypes = ros2TypesForSensor(sensor.sensorType);
  if (expectedTypes.empty())
    return false;

  return std::find(expectedTypes.begin(), expectedTypes.end(),
                   topic.ros2MsgType) != expectedTypes.end();
}

bool hasUnambiguousSensorTokens(const EcmSensorEntry &sensor,
                                const std::string &topic)
{
  const bool sensorToken =
      !sensor.sensorName.empty() && isPathToken(topic, sensor.sensorName);
  const bool modelToken =
      !sensor.modelName.empty() && isPathToken(topic, sensor.modelName);
  const bool linkToken =
      !sensor.linkName.empty() && isPathToken(topic, sensor.linkName);

  return sensorToken || (modelToken && linkToken);
}

std::vector<size_t> collectTypeCompatibleFallbackIndices(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &adv,
    const std::unordered_set<std::string> &claimedTopics)
{
  std::vector<size_t> compatibleIndices;
  for (size_t i = 0; i < adv.size(); ++i)
  {
    const auto &t = adv[i];
    if (claimedTopics.count(t.topicName) > 0)
      continue;
    if (containsDifferentModelPath(t.topicName, sensor.modelName))
      continue;
    if (!t.bridgeable || t.bridgeSpec.empty())
      continue;
    if (!ros2TypeCompatible(sensor, t))
      continue;
    compatibleIndices.push_back(i);
  }

  if (compatibleIndices.empty())
    return {};

  std::vector<size_t> allowedIndices;
  const bool exactlyOneCompatible = compatibleIndices.size() == 1u;

  for (const size_t i : compatibleIndices)
  {
    const auto &t = adv[i];
    if (containsModelPath(t.topicName, sensor.modelName) ||
        hasUnambiguousSensorTokens(sensor, t.topicName) ||
        exactlyOneCompatible)
    {
      allowedIndices.push_back(i);
    }
  }

  return allowedIndices;
}

void appendTopicMatch(
    DiscoveredSensor &sensor,
    const GzTopicEntry &topic,
    std::unordered_set<std::string> &seenSpecs,
    std::unordered_set<std::string> *claimedTopics)
{
  if (!seenSpecs.insert(topic.bridgeSpec).second)
    return;

  sensor.matchedTopicNames.push_back(topic.topicName);
  sensor.matchedBridgeSpecs.push_back(topic.bridgeSpec);
  if (claimedTopics != nullptr)
    claimedTopics->insert(normalizeTopicValue(topic.topicName));
}

// Try to match by sensor/link name token in the topic path combined with
// type compatibility.  Returns NameMatch if any topic matched, else Unresolved.
MatchSource tryNameMatch(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &adv,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  // Name matching requires a known sensor type to avoid spurious matches.
  if (sensor.sensorType.empty()) return MatchSource::Unresolved;

  std::unordered_set<std::string> seenSpecs;
  bool anyFound = false;

  for (const auto &t : adv)
  {
    if (!t.bridgeable || t.bridgeSpec.empty()) continue;
    if (!ros2TypeCompatible(sensor, t)) continue;
    if (!hasUnambiguousSensorTokens(sensor, t.topicName)) continue;

    if (seenSpecs.insert(t.bridgeSpec).second)
    {
      outTopicNames.push_back(t.topicName);
      outBridgeSpecs.push_back(t.bridgeSpec);
      anyFound = true;
    }
  }

  return anyFound ? MatchSource::NameMatch : MatchSource::Unresolved;
}

// Try to match purely by type compatibility (no name check).
// This is the weakest fallback and is intentionally conservative.
MatchSource tryTypeCompatibleFallback(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &adv,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  const auto indices = collectTypeCompatibleFallbackIndices(
      sensor, adv, std::unordered_set<std::string>{});
  if (indices.empty())
    return MatchSource::Unresolved;

  std::unordered_set<std::string> seenSpecs;
  for (const size_t i : indices)
  {
    const auto &t = adv[i];
    if (!seenSpecs.insert(t.bridgeSpec).second)
      continue;
    outTopicNames.push_back(t.topicName);
    outBridgeSpecs.push_back(t.bridgeSpec);
  }
  return outTopicNames.empty()
      ? MatchSource::Unresolved
      : MatchSource::TypeCompatibleFallback;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// tryMatchPrefix
// ---------------------------------------------------------------------------

MatchSource EcmTopicMatcher::tryMatchPrefix(
    const std::string &prefix,
    const std::string &sensorType,
    const std::vector<GzTopicEntry> &advertisedTopics,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  if (prefix.empty()) return MatchSource::Unresolved;

  std::unordered_set<std::string> seenSpecs;
  bool exactPrefixMatch = false;
  bool suffixMatch      = false;

  auto collect = [&](const GzTopicEntry &t, bool isExact) -> bool
  {
    if (!t.bridgeable || t.bridgeSpec.empty()) return false;
    if (!seenSpecs.insert(t.bridgeSpec).second) return false;
    outTopicNames.push_back(t.topicName);
    outBridgeSpecs.push_back(t.bridgeSpec);
    if (isExact) exactPrefixMatch = true;
    else         suffixMatch      = true;
    return true;
  };

  for (const auto &t : advertisedTopics)
  {
    if (t.topicName == prefix)
    {
      collect(t, /*isExact=*/true);
      continue;
    }

    // Sub-path prefix: anything that starts with prefix + "/"
    if (t.topicName.size() > prefix.size() &&
        t.topicName.compare(0, prefix.size(), prefix) == 0 &&
        t.topicName[prefix.size()] == '/')
    {
      collect(t, /*isExact=*/false);
    }
  }

  // Also try explicit type-specific suffix candidates.
  for (const auto &sfx : suffixesForType(sensorType))
  {
    if (sfx.empty()) continue;
    const std::string candidate = prefix + sfx;
    for (const auto &t : advertisedTopics)
    {
      if (t.topicName == candidate)
        collect(t, /*isExact=*/false);
    }
  }

  if (!exactPrefixMatch && !suffixMatch) return MatchSource::Unresolved;
  return exactPrefixMatch ? MatchSource::EcmSensorTopicExact
                          : MatchSource::EcmSensorTopicPrefix;
}

// ---------------------------------------------------------------------------
// matchSensor — four-priority matching cascade
// ---------------------------------------------------------------------------

DiscoveredSensor EcmTopicMatcher::matchSensor(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  DiscoveredSensor result;
  result.sensor = sensor;

  // Priority 1: declaredTopic (SensorTopic ECM component)
  if (!sensor.declaredTopic.empty())
  {
    const GzTopicEntry *exactTopic =
        findExactTopicEntry(sensor.declaredTopic, advertisedTopics);
    result.topicListed = (exactTopic != nullptr);
    if (exactTopic != nullptr)
      result.topicInfoGzType = exactTopic->gzMsgType;

    auto ms = tryMatchPrefix(sensor.declaredTopic, sensor.sensorType,
                             advertisedTopics,
                             result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms != MatchSource::Unresolved)
    {
      result.matchSource = ms;
      result.resolved    = true;
      result.typeSource  = "advertised";
      if (result.topicInfoGzType.empty() && !result.matchedBridgeSpecs.empty())
        result.topicInfoGzType = gzTypeFromBridgeSpec(result.matchedBridgeSpecs.front());

      std::unordered_set<std::string> seenSpecs(result.matchedBridgeSpecs.begin(),
                                                result.matchedBridgeSpecs.end());
      for (const auto &cameraInfo : findAdvertisedCameraInfoTopics(sensor, advertisedTopics))
        appendTopicMatch(result, cameraInfo, seenSpecs, nullptr);
      return result;
    }

    if (exactTopic == nullptr || exactTopic->gzMsgType.empty())
    {
      const auto inferred = inferBridgeTypeForSensor(sensor.sensorType);
      if (!inferred.gzType.empty() && !inferred.ros2Type.empty())
      {
        result.matchedTopicNames.push_back(sensor.declaredTopic);
        result.matchedBridgeSpecs.push_back(
            sensor.declaredTopic + "@" + inferred.ros2Type + "@" + inferred.gzType);
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
    }

    if (exactTopic != nullptr && !exactTopic->gzMsgType.empty())
    {
      result.warning =
          "Sensor Topic type is not bridgeable with ros_gz_bridge.";
      return result;
    }

    result.warning = "Sensor Topic found but type not advertised yet";
    return result;
  }

  // Priority 2: Gazebo standard path (fallbackGazeboTopicPrefix)
  if (!sensor.fallbackGazeboTopicPrefix.empty())
  {
    auto ms = tryMatchPrefix(sensor.fallbackGazeboTopicPrefix, sensor.sensorType,
                             advertisedTopics,
                             result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms != MatchSource::Unresolved)
    {
      result.matchSource = MatchSource::EcmStandardPrefix;
      result.resolved    = true;
      return result;
    }
  }

  // Priority 3: Name match (sensor/link name appears as path token + type compatible)
  {
    auto ms = tryNameMatch(sensor, advertisedTopics,
                           result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms == MatchSource::NameMatch)
    {
      result.matchSource = MatchSource::NameMatch;
      result.resolved    = true;
      result.warning     = "Matched by name — verify this is the correct topic.";
      return result;
    }
  }

  // Priority 4: Type-compatible fallback (weakest; any bridgeable topic of the right type)
  {
    auto ms = tryTypeCompatibleFallback(sensor, advertisedTopics,
                                        result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms == MatchSource::TypeCompatibleFallback)
    {
      result.matchSource = MatchSource::TypeCompatibleFallback;
      result.resolved    = true;
      result.warning     = "Matched by type only — verify or check topics manually.";
      return result;
    }
  }

  // Nothing found — truly unresolved.
  result.warning =
      "No matching topic found. The sensor may not be publishing yet.";
  return result;
}

// ---------------------------------------------------------------------------
// matchAll
//
// Main UI workflow: ECM SensorTopic is the source of truth. For each ECM
// sensor we only attach bridgeable topics that match its declared SensorTopic.
// Sensors without SensorTopic stay unresolved in the main list instead of
// falling back to guessed name/type assignments.
// ---------------------------------------------------------------------------

ModelSensorTree EcmTopicMatcher::matchAll(
    const std::string &worldName,
    const std::vector<EcmSensorEntry> &sensors,
    const std::string &filterModelName,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  ModelSensorTree result;
  result.worldName    = worldName;
  result.modelName    = filterModelName;
  result.ecmConfirmed = true;

  std::vector<DiscoveredSensor> allSensors;
  allSensors.reserve(sensors.size());

  std::unordered_set<std::string> claimedTopics;

  for (const auto &s : sensors)
  {
    DiscoveredSensor sensorResult;
    if (!s.declaredTopic.empty())
    {
      sensorResult = matchSensor(s, advertisedTopics);

      if (sensorResult.resolved)
      {
        const auto matchedTopics = sensorResult.matchedTopicNames;
        const auto matchedSpecs = sensorResult.matchedBridgeSpecs;
        sensorResult.matchedTopicNames.clear();
        sensorResult.matchedBridgeSpecs.clear();

        std::unordered_set<std::string> seenSpecs;
        for (size_t i = 0; i < matchedTopics.size() && i < matchedSpecs.size(); ++i)
        {
          if (claimedTopics.count(normalizeTopicValue(matchedTopics[i])) > 0)
            continue;

          GzTopicEntry topic;
          topic.topicName = matchedTopics[i];
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
    }
    else
    {
      sensorResult.sensor = s;
      sensorResult.warning = "no Sensor Topic in ECM";
    }

    allSensors.push_back(std::move(sensorResult));
  }

  for (size_t i = 0; i < allSensors.size(); ++i)
  {
    auto &sensorResult = allSensors[i];
    if (!sensorResult.resolved && sensorResult.warning.empty())
    {
      sensorResult.warning =
          "No matching topic found. The sensor may not be publishing yet.";
    }

    if (!filterModelName.empty() && sensorResult.sensor.modelName != filterModelName)
      continue;
    result.sensors.push_back(sensorResult);
  }

  return result;
}

}  // namespace gz_ros2_bridge_manager
