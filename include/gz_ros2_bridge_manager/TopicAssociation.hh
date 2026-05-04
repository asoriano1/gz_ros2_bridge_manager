#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "gz_ros2_bridge_manager/EcmTypes.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"

namespace gz_ros2_bridge_manager
{

/// How a sensor's topic was matched against the advertised gz-transport list.
enum class MatchSource
{
  Unresolved = 0,
  EcmSensorTopicPrefix = 1,
  EcmSensorTopicExact = 2,
};

const char *matchSourceName(MatchSource s);

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

/// Pure association logic: maps ECM sensor entries to real advertised
/// gz-transport topics. This component never invents topics.
class TopicAssociation
{
public:
  /// Topic suffixes published by a sensor type; used only to match actual
  /// advertised subtopics beneath an ECM SensorTopic prefix.
  static std::vector<std::string> suffixesForType(const std::string &sensorType);

  /// Normalize a topic string for exact set membership comparisons.
  static std::string normalizeTopic(const std::string &topic);

  /// All real topics claimed by the ECM sensor tree, including any
  /// runtime-derived camera_info siblings that were actually advertised.
  static std::unordered_set<std::string> claimedTopicNames(
      const ModelSensorTree &tree);

  /// Production helper: all bridgeable discovered topics minus the claimed ECM
  /// topic set, using the same normalization path as the UI.
  static std::vector<GzTopicEntry> bridgeableTopicsExcludingClaims(
      const std::vector<GzTopicEntry> &discoveredTopics,
      const ModelSensorTree &tree);

  /// Match one sensor against the full advertised-topic list.
  /// Matching priority:
  ///   1. declaredTopic exact bridgeable topic       → EcmSensorTopicExact
  ///   2. declaredTopic advertised subtopics/prefix  → EcmSensorTopicPrefix
  ///   3. declaredTopic listed without TopicInfo     → exact topic + inferred type
  ///   4. nothing                                    → Unresolved
  static DiscoveredSensor matchSensor(
      const EcmSensorEntry &sensor,
      const std::vector<GzTopicEntry> &advertisedTopics);

  /// Match all sensors with global topic claiming, then optionally filter
  /// the returned tree to a single model name. A real topic already claimed
  /// by one sensor/model will not be attached to another.
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
