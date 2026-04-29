#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

#include <unordered_set>

namespace gz_ros2_bridge_manager
{

// ---------------------------------------------------------------------------
// matchSourceName
// ---------------------------------------------------------------------------

const char *matchSourceName(MatchSource s)
{
  switch (s)
  {
    case MatchSource::EcmSensorTopicExact:  return "EcmSensorTopicExact";
    case MatchSource::EcmSensorTopicPrefix: return "EcmSensorTopicPrefix";
    case MatchSource::EcmStandardPrefix:    return "EcmStandardPrefix";
    case MatchSource::Unresolved:           return "Unresolved";
    default:                                return "Unknown";
  }
}

// ---------------------------------------------------------------------------
// Suffix tables
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
// tryMatchPrefix
//
// Tries to collect advertised topics that match a given prefix:
//   - exact match of the prefix itself
//   - prefix + each type-specific suffix
//   - any topic whose name starts with prefix + "/"
//
// Returns Unresolved if nothing matched.  When something matched:
//   - EcmSensorTopicExact  if the prefix itself was an exact advertised topic
//   - EcmSensorTopicPrefix if only suffix/sub-path matches were found
//
// Callers set the final MatchSource: for fallback-prefix matches the caller
// always overrides to EcmStandardPrefix.
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

  // Also try explicit type-specific suffix candidates
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
// matchSensor
// ---------------------------------------------------------------------------

DiscoveredSensor EcmTopicMatcher::matchSensor(
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  DiscoveredSensor result;
  result.sensor = sensor;

  // Priority 1: declaredTopic (SensorTopic component)
  if (!sensor.declaredTopic.empty())
  {
    auto ms = tryMatchPrefix(sensor.declaredTopic, sensor.sensorType,
                             advertisedTopics,
                             result.matchedTopicNames, result.matchedBridgeSpecs);
    if (ms != MatchSource::Unresolved)
    {
      result.matchSource = ms;  // EcmSensorTopicExact or EcmSensorTopicPrefix
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

  result.warning =
      "Sensor detected in ECM but topic not currently advertised. "
      "Resume simulation or refresh.";
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
