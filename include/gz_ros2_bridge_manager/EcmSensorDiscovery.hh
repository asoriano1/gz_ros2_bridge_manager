#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"

namespace gz_ros2_bridge_manager
{

using EntityId = uint64_t;

/// How a sensor's topic was matched against the advertised gz-transport list.
enum class MatchSource
{
  Unresolved = 0,              // no matching topic found; unresolvable
  EcmStandardPrefix = 1,       // no SensorTopic; matched via Gazebo default path
  EcmSensorTopicPrefix = 2,    // SensorTopic present; topic starts with it (suffix match)
  EcmSensorTopicExact = 3,     // SensorTopic present; exact topic match
  NameMatch = 4,               // sensor/link name appears in topic path (weak, unchecked)
  TypeCompatibleFallback = 5,  // topic type compatible with sensor (weakest, unchecked)
};

const char *matchSourceName(MatchSource s);

/// Raw sensor data extracted from the ECM.
/// Plain C++ only — no Qt, no gz-sim types — to keep this testable standalone.
struct EcmSensorEntry
{
  EntityId modelEntity  = 0;
  std::string modelName;
  bool nestedModel = false;   // true when the direct parent model is itself nested

  EntityId linkEntity   = 0;
  std::string linkName;

  EntityId sensorEntity = 0;
  std::string sensorName;
  std::string sensorType;              // "camera", "gpu_lidar", "imu", …

  std::string declaredTopic;           // SensorTopic component value; empty if absent
  std::string fallbackGazeboTopicPrefix; // /world/<w>/model/<m>/link/<l>/sensor/<s>
};

/// One ECM-confirmed sensor after topic matching.
struct DiscoveredSensor
{
  EcmSensorEntry sensor;
  std::vector<std::string> matchedTopicNames;  // advertised topics
  std::vector<std::string> matchedBridgeSpecs; // bridgeable specs only
  bool resolved = false;
  MatchSource matchSource = MatchSource::Unresolved;
  bool topicListed = false;         // SensorTopic appeared in TopicList
  std::string topicInfoGzType;      // exact TopicInfo type when available
  std::string inferredGzType;       // inferred bridge gz type when TopicInfo missing
  std::string typeSource;           // "advertised" or "type inferred"
  std::string warning;
};

/// Full result for a model (or all models).
struct ModelSensorTree
{
  std::string worldName;
  std::string modelName;
  std::vector<DiscoveredSensor> sensors;
  std::string warning;
  bool ecmConfirmed = false;
};

/// Pure matching logic: maps EcmSensorEntries to advertised gz-transport topics.
/// No gz-sim dependency — testable with plain GzTopicEntry vectors.
class EcmTopicMatcher
{
public:
  /// Topic suffixes published by a sensor type; appended to the declared prefix.
  static std::vector<std::string> suffixesForType(const std::string &sensorType);

  /// Constructs Gazebo's default topic prefix for a sensor when SensorTopic is absent:
  ///   /world/<world>/model/<model>/link/<link>/sensor/<sensor>
  static std::string defaultTopicPrefix(const std::string &worldName,
                                        const EcmSensorEntry &sensor);

  /// Normalize a topic string for exact set membership comparisons.
  static std::string normalizeTopic(const std::string &topic);

  /// All topics claimed by the ECM sensor tree, including declared SensorTopic
  /// values and any matched/derived topics such as camera_info.
  static std::unordered_set<std::string> claimedTopicNames(
      const ModelSensorTree &tree);

  /// Match one sensor against the full advertised-topic list.
  /// Matching priority:
  ///   1. declaredTopic exact match         → EcmSensorTopicExact
  ///   2. declaredTopic + suffix/prefix     → EcmSensorTopicPrefix
  ///   3. fallbackGazeboTopicPrefix matches → EcmStandardPrefix
  ///   4. constrained weak fallback         → NameMatch / TypeCompatibleFallback
  ///   5. nothing                           → Unresolved
  static DiscoveredSensor matchSensor(
      const EcmSensorEntry &sensor,
      const std::vector<GzTopicEntry> &advertisedTopics);

  /// Match all sensors with global topic claiming, then optionally filter
  /// the returned tree to a single model name. A topic already claimed by
  /// one sensor/model will not be attached to another.
  static ModelSensorTree matchAll(
      const std::string &worldName,
      const std::vector<EcmSensorEntry> &sensors,
      const std::string &filterModelName,
      const std::vector<GzTopicEntry> &advertisedTopics);

private:
  /// Try to match a given prefix (exact + suffix expansion + sub-path prefix)
  /// against the advertised list.  Returns MatchSource::Unresolved if nothing found.
  static MatchSource tryMatchPrefix(
      const std::string &prefix,
      const std::string &sensorType,
      const std::vector<GzTopicEntry> &advertisedTopics,
      std::vector<std::string> &outTopicNames,
      std::vector<std::string> &outBridgeSpecs);
};

}  // namespace gz_ros2_bridge_manager
