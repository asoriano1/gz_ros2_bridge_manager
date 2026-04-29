#include <gtest/gtest.h>

#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

// Build a minimal GzTopicEntry with bridgeable status resolved via mapper.
GzTopicEntry makeEntry(const std::string &topic, const std::string &gzType,
                       const BridgeTypeMapper &mapper)
{
  GzTopicEntry e;
  e.topicName   = topic;
  e.gzMsgType   = gzType;
  e.bridgeable  = mapper.isBridgeable(gzType);
  e.ros2MsgType = mapper.ros2Type(gzType);
  e.bridgeSpec  = mapper.bridgeSpec(topic, gzType);
  return e;
}

// Build a sensor with declaredTopic (SensorTopic component).
EcmSensorEntry makeSensor(const std::string &model, const std::string &link,
                          const std::string &sensor, const std::string &type,
                          const std::string &declaredTopic = "")
{
  static EntityId id = 1;
  EcmSensorEntry e;
  e.modelEntity   = id++;
  e.modelName     = model;
  e.linkEntity    = id++;
  e.linkName      = link;
  e.sensorEntity  = id++;
  e.sensorName    = sensor;
  e.sensorType    = type;
  e.declaredTopic = declaredTopic;
  return e;
}

// Build a sensor that has no declaredTopic but has a fallback prefix.
EcmSensorEntry makeSensorWithFallback(const std::string &model,
                                       const std::string &link,
                                       const std::string &sensor,
                                       const std::string &type,
                                       const std::string &fallbackPrefix)
{
  EcmSensorEntry e = makeSensor(model, link, sensor, type);
  e.fallbackGazeboTopicPrefix = fallbackPrefix;
  return e;
}

}  // namespace

// ============================================================================
// Test 1 — Exact topic match (SensorTopic == advertised topic)
// ============================================================================
TEST(EcmTopicMatcher, ExactTopicMatch)
{
  BridgeTypeMapper mapper;
  const std::string topic =
      "/world/default/model/robot/link/base/sensor/lidar/scan";
  GzTopicEntry adv = makeEntry(topic, "gz.msgs.LaserScan", mapper);

  EcmSensorEntry s = makeSensor("robot", "base", "lidar", "gpu_lidar", topic);

  auto ds = EcmTopicMatcher::matchSensor(s, {adv});
  EXPECT_TRUE(ds.resolved);
  ASSERT_EQ(ds.matchedBridgeSpecs.size(), 1u);
  EXPECT_NE(ds.matchedBridgeSpecs[0].find("/scan"), std::string::npos);
}

// ============================================================================
// Test 2 — Declared topic is prefix; camera emits image + camera_info
// ============================================================================
TEST(EcmTopicMatcher, CameraPrefixMatchesImageAndCameraInfo)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/cam_link/sensor/front_cam";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/image",       "gz.msgs.Image",        mapper),
    makeEntry(prefix + "/camera_info", "gz.msgs.CameraInfo",   mapper),
    makeEntry(prefix + "/depth",       "gz.msgs.Image",        mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "cam_link", "front_cam", "camera", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  EXPECT_GE(ds.matchedBridgeSpecs.size(), 2u);

  auto hasSpec = [&](const std::string &sub)
  {
    for (const auto &spec : ds.matchedBridgeSpecs)
      if (spec.find(sub) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(hasSpec("/image"));
  EXPECT_TRUE(hasSpec("/camera_info"));
}

// ============================================================================
// Test 3 — Lidar prefix → scan suffix matched
// ============================================================================
TEST(EcmTopicMatcher, LidarPrefixMatchesScan)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/base/sensor/front_laser";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/scan",   "gz.msgs.LaserScan", mapper),
    makeEntry(prefix + "/points", "gz.msgs.PointCloudPacked", mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "base", "front_laser", "gpu_lidar", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);

  bool hasScan = false;
  for (const auto &spec : ds.matchedBridgeSpecs)
    if (spec.find("/scan") != std::string::npos) hasScan = true;
  EXPECT_TRUE(hasScan);
}

// ============================================================================
// Test 4 — IMU prefix matched via /imu suffix
// ============================================================================
TEST(EcmTopicMatcher, ImuPrefixMatchesImuSuffix)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/imu_link/sensor/imu_sensor";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/imu", "gz.msgs.IMU", mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "imu_link", "imu_sensor", "imu", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchedBridgeSpecs.size(), 1u);
}

// ============================================================================
// Test 5 — No advertised topic → unresolved with warning
// ============================================================================
TEST(EcmTopicMatcher, NoAdvertisedTopicIsUnresolved)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/base/sensor/ghost";

  EcmSensorEntry s = makeSensor("robot", "base", "ghost", "lidar", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, {});
  EXPECT_FALSE(ds.resolved);
  EXPECT_FALSE(ds.warning.empty());
}

// ============================================================================
// Test 6 — ECM sensor match wins: matchedBridgeSpecs contains the ECM topic,
//           not any heuristic topic that might happen to have the same prefix
// ============================================================================
TEST(EcmTopicMatcher, EcmMatchedTopicIsInBridgeSpecs)
{
  BridgeTypeMapper mapper;
  const std::string ecmPrefix =
      "/world/default/model/robot/link/base/sensor/lidar";
  const std::string ecmTopic = ecmPrefix + "/scan";

  std::vector<GzTopicEntry> adv{
    makeEntry(ecmTopic,          "gz.msgs.LaserScan",    mapper),
    makeEntry("/random/other",   "gz.msgs.LaserScan",    mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "base", "lidar", "gpu_lidar", ecmPrefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);

  for (const auto &topic : ds.matchedTopicNames)
    EXPECT_EQ(topic, ecmTopic) << "should only match the ECM-declared prefix";
}

// ============================================================================
// Test 7 — matchAll filters by model name
// ============================================================================
TEST(EcmTopicMatcher, MatchAllFiltersToSelectedModel)
{
  BridgeTypeMapper mapper;
  const std::string topicA =
      "/world/default/model/A/link/la/sensor/sa/scan";
  const std::string topicB =
      "/world/default/model/B/link/lb/sensor/sb/scan";

  std::vector<GzTopicEntry> adv{
    makeEntry(topicA, "gz.msgs.LaserScan", mapper),
    makeEntry(topicB, "gz.msgs.LaserScan", mapper),
  };

  std::vector<EcmSensorEntry> sensors{
    makeSensor("A", "la", "sa", "gpu_lidar",
               "/world/default/model/A/link/la/sensor/sa"),
    makeSensor("B", "lb", "sb", "gpu_lidar",
               "/world/default/model/B/link/lb/sensor/sb"),
  };

  auto tree = EcmTopicMatcher::matchAll("default", sensors, "A", adv);
  ASSERT_EQ(tree.sensors.size(), 1u);
  EXPECT_EQ(tree.sensors[0].sensor.modelName, "A");
  EXPECT_TRUE(tree.sensors[0].resolved);
}

// ============================================================================
// Test 8 — Duplicate bridge specs are removed
// ============================================================================
TEST(EcmTopicMatcher, DuplicateBridgeSpecsRemoved)
{
  BridgeTypeMapper mapper;
  const std::string prefix = "/world/default/model/robot/link/base/sensor/cam";
  const std::string imageTopic = prefix + "/image";

  // Both "exact" prefix and "image suffix" would match imageTopic — should appear once.
  std::vector<GzTopicEntry> adv{
    makeEntry(imageTopic, "gz.msgs.Image", mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "base", "cam", "camera", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);

  // Count occurrences of the image bridge spec.
  size_t count = 0;
  const std::string imageSpec = mapper.bridgeSpec(imageTopic, "gz.msgs.Image");
  for (const auto &spec : ds.matchedBridgeSpecs)
    if (spec == imageSpec) ++count;
  EXPECT_EQ(count, 1u) << "image spec must appear exactly once";
}

// ============================================================================
// Test 9 — defaultTopicPrefix constructs Gazebo standard path
// ============================================================================
TEST(EcmTopicMatcher, DefaultTopicPrefixFormat)
{
  EcmSensorEntry s;
  s.modelName  = "robot";
  s.linkName   = "base_link";
  s.sensorName = "front_cam";

  const std::string expected =
      "/world/myworld/model/robot/link/base_link/sensor/front_cam";
  EXPECT_EQ(EcmTopicMatcher::defaultTopicPrefix("myworld", s), expected);
}

// ============================================================================
// Test 10 — Empty sensor names produce empty prefix (no crash)
// ============================================================================
TEST(EcmTopicMatcher, EmptyNamesGiveEmptyPrefix)
{
  EcmSensorEntry s;
  EXPECT_TRUE(EcmTopicMatcher::defaultTopicPrefix("world", s).empty());
}

// ============================================================================
// Test 11 — declaredTopic exact match → MatchSource::EcmSensorTopicExact
// ============================================================================
TEST(EcmTopicMatcher, MatchSourceExactWhenDeclaredTopicMatchesDirectly)
{
  BridgeTypeMapper mapper;
  const std::string fullTopic =
      "/world/default/model/robot/link/base/sensor/lidar/scan";

  GzTopicEntry adv = makeEntry(fullTopic, "gz.msgs.LaserScan", mapper);
  EcmSensorEntry s = makeSensor("robot", "base", "lidar", "gpu_lidar", fullTopic);

  auto ds = EcmTopicMatcher::matchSensor(s, {adv});
  EXPECT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmSensorTopicExact);
}

// ============================================================================
// Test 12 — declaredTopic is a prefix, topics via suffix → EcmSensorTopicPrefix
// ============================================================================
TEST(EcmTopicMatcher, MatchSourcePrefixWhenSuffixExpanded)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/base/sensor/front_cam";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/image",       "gz.msgs.Image",      mapper),
    makeEntry(prefix + "/camera_info", "gz.msgs.CameraInfo", mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "base", "front_cam", "camera", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmSensorTopicPrefix);
}

// ============================================================================
// Test 13 — No declaredTopic, fallbackGazeboTopicPrefix used → EcmStandardPrefix
// ============================================================================
TEST(EcmTopicMatcher, MatchSourceStandardPrefixWhenFallbackUsed)
{
  BridgeTypeMapper mapper;
  const std::string fallback =
      "/world/default/model/robot/link/base/sensor/lidar";

  std::vector<GzTopicEntry> adv{
    makeEntry(fallback + "/scan", "gz.msgs.LaserScan", mapper),
  };

  // No declaredTopic — only fallback prefix set.
  EcmSensorEntry s = makeSensorWithFallback("robot", "base", "lidar", "gpu_lidar", fallback);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmStandardPrefix);
}

// ============================================================================
// Test 14 — declaredTopic takes priority over fallback when both could match
// ============================================================================
TEST(EcmTopicMatcher, DeclaredTopicTakesPriorityOverFallback)
{
  BridgeTypeMapper mapper;
  const std::string declared = "/custom/lidar";
  const std::string fallback =
      "/world/default/model/robot/link/base/sensor/lidar";

  std::vector<GzTopicEntry> adv{
    makeEntry(declared + "/scan", "gz.msgs.LaserScan", mapper),
    makeEntry(fallback + "/scan", "gz.msgs.LaserScan", mapper),
  };

  EcmSensorEntry s = makeSensor("robot", "base", "lidar", "gpu_lidar", declared);
  s.fallbackGazeboTopicPrefix = fallback;

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  // declaredTopic matched first → EcmSensorTopicPrefix (not EcmStandardPrefix)
  EXPECT_NE(ds.matchSource, MatchSource::EcmStandardPrefix);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmSensorTopicPrefix);
}

// ============================================================================
// Test 15 — No match anywhere → matchSource stays Unresolved
// ============================================================================
TEST(EcmTopicMatcher, MatchSourceUnresolvedWhenNoTopicsMatch)
{
  EcmSensorEntry s = makeSensorWithFallback(
      "robot", "base", "ghost", "gpu_lidar",
      "/world/default/model/robot/link/base/sensor/ghost");
  s.declaredTopic = "/custom/ghost";

  auto ds = EcmTopicMatcher::matchSensor(s, {});
  EXPECT_FALSE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::Unresolved);
}

// ============================================================================
// Test 16 — nestedModel flag is carried through; matching still works
// ============================================================================
TEST(EcmTopicMatcher, NestedModelFlagDoesNotBreakMatching)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/parent/link/base/sensor/lidar";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/scan", "gz.msgs.LaserScan", mapper),
  };

  EcmSensorEntry s = makeSensor("parent", "base", "lidar", "gpu_lidar", prefix);
  s.nestedModel = true;

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  EXPECT_TRUE(ds.sensor.nestedModel);
}

// ============================================================================
// Test 17 — matchSourceName returns correct strings
// ============================================================================
TEST(EcmTopicMatcher, MatchSourceNameStrings)
{
  EXPECT_STREQ(matchSourceName(MatchSource::Unresolved),           "Unresolved");
  EXPECT_STREQ(matchSourceName(MatchSource::EcmStandardPrefix),    "EcmStandardPrefix");
  EXPECT_STREQ(matchSourceName(MatchSource::EcmSensorTopicPrefix), "EcmSensorTopicPrefix");
  EXPECT_STREQ(matchSourceName(MatchSource::EcmSensorTopicExact),  "EcmSensorTopicExact");
}

// ============================================================================
// Test 18 — matchAll with empty filterModelName returns all sensors
// ============================================================================
TEST(EcmTopicMatcher, MatchAllEmptyFilterReturnsAll)
{
  BridgeTypeMapper mapper;
  const std::string prefixA = "/world/w/model/A/link/la/sensor/sa";
  const std::string prefixB = "/world/w/model/B/link/lb/sensor/sb";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefixA + "/scan", "gz.msgs.LaserScan", mapper),
    makeEntry(prefixB + "/scan", "gz.msgs.LaserScan", mapper),
  };

  std::vector<EcmSensorEntry> sensors{
    makeSensor("A", "la", "sa", "gpu_lidar", prefixA),
    makeSensor("B", "lb", "sb", "gpu_lidar", prefixB),
  };

  auto tree = EcmTopicMatcher::matchAll("w", sensors, "", adv);
  EXPECT_EQ(tree.sensors.size(), 2u);
  EXPECT_TRUE(tree.ecmConfirmed);
}

// ============================================================================
// Test 19 — Fallback prefix with empty model/link/sensor names yields no match
// ============================================================================
TEST(EcmTopicMatcher, EmptyFallbackPrefixAndNoDeclaredTopicIsUnresolved)
{
  BridgeTypeMapper mapper;
  const std::string topic = "/world/w/model/robot/link/base/sensor/lidar/scan";

  std::vector<GzTopicEntry> adv{
    makeEntry(topic, "gz.msgs.LaserScan", mapper),
  };

  // Sensor with neither declaredTopic nor fallback set.
  EcmSensorEntry s;
  s.sensorName = "lidar";
  s.sensorType = "gpu_lidar";

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_FALSE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::Unresolved);
}

// ============================================================================
// Test 20 — Camera: all known suffixes collected via sub-path scan
// ============================================================================
TEST(EcmTopicMatcher, CameraSubPathCollectsAllSuffixes)
{
  BridgeTypeMapper mapper;
  const std::string prefix = "/world/w/model/r/link/l/sensor/cam";

  std::vector<GzTopicEntry> adv{
    makeEntry(prefix + "/image",       "gz.msgs.Image",      mapper),
    makeEntry(prefix + "/camera_info", "gz.msgs.CameraInfo", mapper),
    makeEntry(prefix + "/depth",       "gz.msgs.Image",      mapper),
    makeEntry(prefix + "/depth_raw",   "gz.msgs.Image",      mapper),
    makeEntry(prefix + "/points",      "gz.msgs.PointCloudPacked", mapper),
  };

  EcmSensorEntry s = makeSensor("r", "l", "cam", "depth_camera", prefix);

  auto ds = EcmTopicMatcher::matchSensor(s, adv);
  EXPECT_TRUE(ds.resolved);
  // All five topics must be collected (no duplicates).
  EXPECT_EQ(ds.matchedTopicNames.size(), 5u);
  EXPECT_EQ(ds.matchedBridgeSpecs.size(), 5u);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
