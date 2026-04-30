#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

#include <algorithm>
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

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

namespace
{

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

// Try to match by sensor/link name token in the topic path combined with
// type compatibility.  Returns NameMatch if any topic matched, else Unresolved.
MatchSource tryNameMatch(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &adv,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  // Name matching requires a known sensor type to avoid spurious matches.
  const auto expectedTypes = ros2TypesForSensor(sensor.sensorType);
  if (expectedTypes.empty()) return MatchSource::Unresolved;
  if (sensor.sensorName.empty()) return MatchSource::Unresolved;

  std::unordered_set<std::string> seenSpecs;
  bool anyFound = false;

  for (const auto &t : adv)
  {
    if (!t.bridgeable || t.bridgeSpec.empty()) continue;

    // Name condition: sensor name or link name appears as a path segment.
    const bool nameFit = isPathToken(t.topicName, sensor.sensorName) ||
                         (!sensor.linkName.empty() &&
                          isPathToken(t.topicName, sensor.linkName));
    if (!nameFit) continue;

    // Type condition: topic's ROS 2 type must be compatible with sensor type.
    const bool typeFit =
        std::find(expectedTypes.begin(), expectedTypes.end(),
                  t.ros2MsgType) != expectedTypes.end();
    if (!typeFit) continue;

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
// This is the weakest fallback: collect ALL bridgeable topics whose ROS 2
// type is expected for this sensor type.
MatchSource tryTypeCompatibleFallback(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &adv,
    std::vector<std::string> &outTopicNames,
    std::vector<std::string> &outBridgeSpecs)
{
  const auto expectedTypes = ros2TypesForSensor(sensor.sensorType);
  if (expectedTypes.empty()) return MatchSource::Unresolved;

  std::unordered_set<std::string> seenSpecs;
  bool anyFound = false;

  for (const auto &t : adv)
  {
    if (!t.bridgeable || t.bridgeSpec.empty()) continue;
    const bool typeFit =
        std::find(expectedTypes.begin(), expectedTypes.end(),
                  t.ros2MsgType) != expectedTypes.end();
    if (!typeFit) continue;

    if (seenSpecs.insert(t.bridgeSpec).second)
    {
      outTopicNames.push_back(t.topicName);
      outBridgeSpecs.push_back(t.bridgeSpec);
      anyFound = true;
    }
  }

  return anyFound ? MatchSource::TypeCompatibleFallback : MatchSource::Unresolved;
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
    auto ms = tryMatchPrefix(sensor.declaredTopic, sensor.sensorType,
                             advertisedTopics,
                             result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms != MatchSource::Unresolved)
    {
      result.matchSource = ms;
      result.resolved    = true;
      return result;
    }
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

  for (const auto &s : sensors)
  {
    if (!filterModelName.empty() && s.modelName != filterModelName)
      continue;
    result.sensors.push_back(matchSensor(s, advertisedTopics));
  }
  return result;
}

}  // namespace gz_ros2_bridge_manager
