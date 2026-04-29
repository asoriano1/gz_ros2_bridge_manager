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

  auto ds = EcmTopicMatcher::matchSensor("default", s, {adv});
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, adv);
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, adv);
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, adv);
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, {});
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, adv);
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

  auto ds = EcmTopicMatcher::matchSensor("default", s, adv);
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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
