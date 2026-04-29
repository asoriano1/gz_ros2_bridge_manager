#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

#include <unordered_set>

namespace gz_ros2_bridge_manager
{

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
  // Unknown type — try the prefix itself (exact) and a bare empty suffix.
  return {""};
}

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

DiscoveredSensor EcmTopicMatcher::matchSensor(
    const std::string &worldName,
    const EcmSensorEntry &sensor,
    const std::vector<GzTopicEntry> &advertisedTopics)
{
  DiscoveredSensor result;
  result.sensor = sensor;

  // Authoritative: SensorTopic component. Fallback: Gazebo default path.
  const std::string prefix = sensor.declaredTopic.empty()
      ? defaultTopicPrefix(worldName, sensor)
      : sensor.declaredTopic;

  if (prefix.empty())
  {
    result.warning =
        "Cannot determine topic prefix: no SensorTopic component and "
        "incomplete ECM hierarchy data (model/link/sensor name missing).";
    return result;
  }

  // Candidate topic names to try:
  //   1. The prefix itself (exact match for single-topic sensors).
  //   2. prefix + each type-specific suffix.
  std::vector<std::string> candidates{prefix};
  for (const auto &sfx : suffixesForType(sensor.sensorType))
  {
    if (!sfx.empty())
      candidates.push_back(prefix + sfx);
  }

  std::unordered_set<std::string> seenSpecs;
  for (const auto &candidate : candidates)
  {
    for (const auto &t : advertisedTopics)
    {
      // Accept: exact match OR any topic whose name starts with prefix + "/".
      const bool match =
          (t.topicName == candidate) ||
          (candidate == prefix &&
           t.topicName.size() > prefix.size() &&
           t.topicName.compare(0, prefix.size(), prefix) == 0 &&
           t.topicName[prefix.size()] == '/');

      if (match && t.bridgeable && !t.bridgeSpec.empty())
      {
        if (seenSpecs.insert(t.bridgeSpec).second)
        {
          result.matchedBridgeSpecs.push_back(t.bridgeSpec);
          result.matchedTopicNames.push_back(t.topicName);
          result.resolved = true;
        }
      }
    }
  }

  if (!result.resolved)
  {
    result.warning =
        "Sensor detected in ECM but topic not currently advertised. "
        "Resume simulation or refresh.";
  }
  return result;
}

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
    result.sensors.push_back(matchSensor(worldName, s, advertisedTopics));
  }
  return result;
}

}  // namespace gz_ros2_bridge_manager
