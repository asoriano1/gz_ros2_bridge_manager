#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"

namespace gz_ros2_bridge_manager
{

using EntityId = uint64_t;

/// Raw sensor information extracted from the ECM.
/// Plain C++ — no Qt, no gz-sim headers — to keep this testable standalone.
struct EcmSensorEntry
{
  EntityId modelEntity  = 0;
  std::string modelName;
  EntityId linkEntity   = 0;
  std::string linkName;
  EntityId sensorEntity = 0;
  std::string sensorName;
  std::string sensorType;     // "camera", "gpu_lidar", "imu", …
  std::string declaredTopic;  // SensorTopic component value; may be empty
};

/// One ECM-confirmed sensor after topic matching.
struct DiscoveredSensor
{
  EcmSensorEntry sensor;
  std::vector<std::string> matchedTopicNames;  // actually advertised topics
  std::vector<std::string> matchedBridgeSpecs; // bridgeable specs only
  bool resolved = false;   // ≥1 topic matched
  std::string warning;
};

/// Full ECM-confirmed sensor result for a model (or all models).
struct ModelSensorTree
{
  std::string worldName;
  std::string modelName;
  std::vector<DiscoveredSensor> sensors;
  std::string warning;
  bool ecmConfirmed = false;  // true when data came from a live ECM Update()
};

/// Pure matching logic that maps EcmSensorEntries against advertised gz-transport
/// topics.  This class has NO gz-sim dependency and can be unit-tested with plain
/// GzTopicEntry vectors.
class EcmTopicMatcher
{
public:
  // Topic suffixes published by a sensor type (appended to the declared prefix).
  // Empty string means try the prefix itself as an exact topic.
  static std::vector<std::string> suffixesForType(const std::string &sensorType);

  // Constructs Gazebo's default topic prefix for a sensor when SensorTopic is absent:
  //   /world/<world>/model/<model>/link/<link>/sensor/<sensor>
  static std::string defaultTopicPrefix(const std::string &worldName,
                                        const EcmSensorEntry &sensor);

  // Match one sensor against the full advertised-topic list.
  static DiscoveredSensor matchSensor(
      const std::string &worldName,
      const EcmSensorEntry &sensor,
      const std::vector<GzTopicEntry> &advertisedTopics);

  // Match all sensors, optionally filtering to a single model name.
  // Populates `allMatchedTopics` for exclusion from heuristic matching.
  static ModelSensorTree matchAll(
      const std::string &worldName,
      const std::vector<EcmSensorEntry> &sensors,
      const std::string &filterModelName,
      const std::vector<GzTopicEntry> &advertisedTopics);
};

}  // namespace gz_ros2_bridge_manager
