#include <gtest/gtest.h>

#include <algorithm>

#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/TopicAssociation.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

GzTopicEntry makeEntry(const std::string &topic, const std::string &gzType,
                       const BridgeTypeMapper &mapper)
{
  GzTopicEntry entry;
  entry.topicName = topic;
  entry.gzMsgType = gzType;
  entry.bridgeable = mapper.isBridgeable(gzType);
  entry.ros2MsgType = mapper.ros2Type(gzType);
  entry.bridgeSpec = mapper.bridgeSpec(topic, gzType);
  return entry;
}

GzTopicEntry makeListedTopicWithoutType(const std::string &topic)
{
  GzTopicEntry entry;
  entry.topicName = topic;
  return entry;
}

bool hasTopic(const std::vector<std::string> &topics, const std::string &topic)
{
  return std::find(topics.begin(), topics.end(), topic) != topics.end();
}

std::vector<std::string> additionalTopicsAfterClaims(
    const std::vector<GzTopicEntry> &advertisedTopics,
    const ModelSensorTree &tree)
{
  std::vector<std::string> topics;
  for (const auto &entry :
       TopicAssociation::bridgeableTopicsExcludingClaims(advertisedTopics, tree))
  {
    topics.push_back(TopicAssociation::normalizeTopic(entry.topicName));
  }
  return topics;
}

EcmSensorEntry makeSensor(const std::string &model,
                          const std::string &link,
                          const std::string &sensor,
                          const std::string &type,
                          const std::string &declaredTopic = "")
{
  static EntityId id = 1;
  EcmSensorEntry entry;
  entry.modelEntity = id++;
  entry.modelName = model;
  entry.linkEntity = id++;
  entry.linkName = link;
  entry.sensorEntity = id++;
  entry.sensorName = sensor;
  entry.sensorType = type;
  entry.declaredTopic = declaredTopic;
  return entry;
}

}  // namespace

TEST(TopicAssociation, ExactTopicMatch)
{
  BridgeTypeMapper mapper;
  const std::string topic =
      "/world/default/model/robot/link/base/sensor/lidar/scan";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "lidar", "gpu_lidar", topic),
      {makeEntry(topic, "gz.msgs.LaserScan", mapper)});

  ASSERT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmSensorTopicExact);
  ASSERT_EQ(ds.matchedBridgeSpecs.size(), 1u);
  EXPECT_EQ(ds.matchedBridgeSpecs[0],
            topic + "@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
}

TEST(TopicAssociation, CameraPrefixMatchesAdvertisedImageAndCameraInfo)
{
  BridgeTypeMapper mapper;
  const std::string prefix =
      "/world/default/model/robot/link/cam_link/sensor/front_cam";

  std::vector<GzTopicEntry> advertisedTopics{
    makeEntry(prefix + "/image", "gz.msgs.Image", mapper),
    makeEntry(prefix + "/camera_info", "gz.msgs.CameraInfo", mapper),
    makeEntry(prefix + "/depth", "gz.msgs.Image", mapper),
  };

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "cam_link", "front_cam", "camera", prefix),
      advertisedTopics);

  ASSERT_TRUE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::EcmSensorTopicPrefix);
  EXPECT_TRUE(hasTopic(ds.matchedTopicNames,
      TopicAssociation::normalizeTopic(prefix + "/image")));
  EXPECT_TRUE(hasTopic(ds.matchedTopicNames,
      TopicAssociation::normalizeTopic(prefix + "/camera_info")));
}

TEST(TopicAssociation, ImuSensorTopicWithoutTopicInfoUsesInferredImuType)
{
  const std::string topic = "/sensor_test_robot_urdf_1/imu/data_raw";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "imu_sensor", "imu", topic),
      {makeListedTopicWithoutType(topic)});

  ASSERT_TRUE(ds.resolved);
  EXPECT_TRUE(ds.topicListed);
  EXPECT_EQ(ds.typeSource, "type inferred");
  EXPECT_EQ(ds.inferredGzType, "gz.msgs.IMU");
  ASSERT_EQ(ds.matchedBridgeSpecs.size(), 1u);
  EXPECT_EQ(ds.matchedBridgeSpecs[0],
            topic + "@sensor_msgs/msg/Imu@gz.msgs.IMU");
}

TEST(TopicAssociation, CameraSensorTopicWithoutTopicInfoUsesInferredImageType)
{
  const std::string topic = "/sensor_test_robot_urdf_1/camera/image_raw";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "camera_sensor", "camera", topic),
      {makeListedTopicWithoutType(topic)});

  ASSERT_TRUE(ds.resolved);
  EXPECT_TRUE(ds.topicListed);
  EXPECT_EQ(ds.typeSource, "type inferred");
  EXPECT_EQ(ds.inferredGzType, "gz.msgs.Image");
  EXPECT_EQ(ds.warning,
            "Topic type inferred from ECM sensor type; TopicInfo not available yet.");
}

TEST(TopicAssociation, LidarSensorTopicWithoutTopicInfoUsesInferredLaserScanType)
{
  const std::string topic = "/sensor_test_robot_urdf_1/lidar";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "lidar", "gpu_lidar", topic),
      {makeListedTopicWithoutType(topic)});

  ASSERT_TRUE(ds.resolved);
  EXPECT_EQ(ds.inferredGzType, "gz.msgs.LaserScan");
  ASSERT_EQ(ds.matchedBridgeSpecs.size(), 1u);
  EXPECT_EQ(ds.matchedBridgeSpecs[0],
            topic + "@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
}

TEST(TopicAssociation, AdvertisedTopicInfoTakesPrecedenceOverInference)
{
  BridgeTypeMapper mapper;
  const std::string topic = "/sensor_test_robot_urdf_1/camera/image_raw";

  std::vector<GzTopicEntry> advertisedTopics{
    makeListedTopicWithoutType(topic),
    makeEntry(topic, "gz.msgs.Image", mapper),
  };

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "camera_sensor", "camera", topic),
      advertisedTopics);

  ASSERT_TRUE(ds.resolved);
  EXPECT_EQ(ds.typeSource, "advertised");
  EXPECT_EQ(ds.topicInfoGzType, "gz.msgs.Image");
  EXPECT_TRUE(ds.inferredGzType.empty());
}

TEST(TopicAssociation, SensorTopicNotAdvertisedDoesNotCreateBridgeSpec)
{
  const std::string topic = "/robot/front_camera";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "front_camera", "camera", topic),
      {});

  EXPECT_FALSE(ds.resolved);
  EXPECT_FALSE(ds.topicListed);
  EXPECT_TRUE(ds.matchedTopicNames.empty());
  EXPECT_TRUE(ds.matchedBridgeSpecs.empty());
  EXPECT_EQ(ds.warning, "Sensor Topic not advertised in Gazebo Transport yet.");
}

TEST(TopicAssociation, SensorWithoutSensorTopicStaysUnresolved)
{
  BridgeTypeMapper mapper;
  const std::string topic = "/robot/front_laser/scan";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "front_laser", "gpu_lidar"),
      {makeEntry(topic, "gz.msgs.LaserScan", mapper)});

  EXPECT_FALSE(ds.resolved);
  EXPECT_EQ(ds.matchSource, MatchSource::Unresolved);
  EXPECT_TRUE(ds.matchedTopicNames.empty());
  EXPECT_EQ(ds.warning, "no Sensor Topic in ECM");
}

TEST(TopicAssociation, WeakPathLikeTopicIsNotTreatedAsDiscovered)
{
  BridgeTypeMapper mapper;
  const std::string weakTopic = "/robot_ns/front_laser/scan";

  std::vector<GzTopicEntry> advertisedTopics{
    makeEntry(weakTopic, "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll(
      "default",
      {makeSensor("robot_ns", "base", "front_laser", "gpu_lidar")},
      "",
      advertisedTopics);

  ASSERT_EQ(tree.sensors.size(), 1u);
  EXPECT_FALSE(tree.sensors[0].resolved);
  EXPECT_TRUE(tree.sensors[0].matchedTopicNames.empty());

  const auto additionalTopics = additionalTopicsAfterClaims(advertisedTopics, tree);
  ASSERT_EQ(additionalTopics.size(), 1u);
  EXPECT_EQ(additionalTopics[0], weakTopic);
}

TEST(TopicAssociation, ClaimedTopicDoesNotLeakAcrossModels)
{
  BridgeTypeMapper mapper;
  const std::string topicA =
      "/world/default/model/A/link/la/sensor/laser_a/scan";

  std::vector<GzTopicEntry> advertisedTopics{
    makeEntry(topicA, "gz.msgs.LaserScan", mapper),
  };

  std::vector<EcmSensorEntry> sensors{
    makeSensor("A", "la", "laser_a", "gpu_lidar",
               "/world/default/model/A/link/la/sensor/laser_a"),
    makeSensor("B", "lb", "laser_b", "gpu_lidar",
               "/world/default/model/B/link/lb/sensor/laser_b"),
  };

  auto tree = TopicAssociation::matchAll("default", sensors, "", advertisedTopics);
  ASSERT_EQ(tree.sensors.size(), 2u);

  EXPECT_TRUE(tree.sensors[0].resolved);
  EXPECT_FALSE(tree.sensors[1].resolved);
  EXPECT_TRUE(tree.sensors[1].matchedTopicNames.empty());
}

TEST(TopicAssociation, MatchAllFiltersToSelectedModel)
{
  BridgeTypeMapper mapper;
  const std::string topicA =
      "/world/default/model/A/link/la/sensor/sa/scan";
  const std::string topicB =
      "/world/default/model/B/link/lb/sensor/sb/scan";

  std::vector<EcmSensorEntry> sensors{
    makeSensor("A", "la", "sa", "gpu_lidar",
               "/world/default/model/A/link/la/sensor/sa"),
    makeSensor("B", "lb", "sb", "gpu_lidar",
               "/world/default/model/B/link/lb/sensor/sb"),
  };

  std::vector<GzTopicEntry> advertisedTopics{
    makeEntry(topicA, "gz.msgs.LaserScan", mapper),
    makeEntry(topicB, "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll("default", sensors, "A", advertisedTopics);
  ASSERT_EQ(tree.sensors.size(), 1u);
  EXPECT_EQ(tree.sensors[0].sensor.modelName, "A");
}

TEST(TopicAssociation, MatchAllEmptyFilterReturnsAll)
{
  BridgeTypeMapper mapper;
  const std::string prefixA = "/world/w/model/A/link/la/sensor/sa";
  const std::string prefixB = "/world/w/model/B/link/lb/sensor/sb";

  std::vector<EcmSensorEntry> sensors{
    makeSensor("A", "la", "sa", "gpu_lidar", prefixA),
    makeSensor("B", "lb", "sb", "gpu_lidar", prefixB),
  };

  std::vector<GzTopicEntry> advertisedTopics{
    makeEntry(prefixA + "/scan", "gz.msgs.LaserScan", mapper),
    makeEntry(prefixB + "/scan", "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll("w", sensors, "", advertisedTopics);
  EXPECT_EQ(tree.sensors.size(), 2u);
  EXPECT_TRUE(tree.ecmConfirmed);
}

TEST(TopicAssociation, MatchSourceNameStrings)
{
  EXPECT_STREQ(matchSourceName(MatchSource::Unresolved), "Unresolved");
  EXPECT_STREQ(matchSourceName(MatchSource::EcmSensorTopicPrefix), "ECM prefix");
  EXPECT_STREQ(matchSourceName(MatchSource::EcmSensorTopicExact), "ECM exact");
}

TEST(TopicAssociation, DuplicateBridgeSpecsAreRemoved)
{
  BridgeTypeMapper mapper;
  const std::string prefix = "/world/default/model/robot/link/base/sensor/cam";
  const std::string imageTopic = prefix + "/image";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "cam", "camera", prefix),
      {makeEntry(imageTopic, "gz.msgs.Image", mapper)});

  ASSERT_TRUE(ds.resolved);
  const std::string imageSpec = mapper.bridgeSpec(imageTopic, "gz.msgs.Image");
  EXPECT_EQ(std::count(ds.matchedBridgeSpecs.begin(),
                       ds.matchedBridgeSpecs.end(),
                       imageSpec), 1);
}

TEST(TopicAssociation, AdvertisedCameraInfoSiblingIsAttachedToCameraSensor)
{
  BridgeTypeMapper mapper;
  const std::string imageTopic = "/sensor_test_robot_urdf_1/camera/image_raw";
  const std::string cameraInfoTopic = "/sensor_test_robot_urdf_1/camera/camera_info";

  std::vector<GzTopicEntry> advertisedTopics{
    makeListedTopicWithoutType(imageTopic),
    makeEntry(cameraInfoTopic, "gz.msgs.CameraInfo", mapper),
  };

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "camera_sensor", "camera", imageTopic),
      advertisedTopics);

  ASSERT_TRUE(ds.resolved);
  EXPECT_TRUE(hasTopic(ds.matchedTopicNames,
      TopicAssociation::normalizeTopic(cameraInfoTopic)));
}

TEST(TopicAssociation, NonAdvertisedCameraInfoIsNotInvented)
{
  const std::string imageTopic = "/sensor_test_robot_urdf_1/camera/image_raw";

  auto ds = TopicAssociation::matchSensor(
      makeSensor("robot", "base", "camera_sensor", "camera", imageTopic),
      {makeListedTopicWithoutType(imageTopic)});

  ASSERT_TRUE(ds.resolved);
  ASSERT_EQ(ds.matchedTopicNames.size(), 1u);
  EXPECT_EQ(ds.matchedTopicNames[0], imageTopic);
}

TEST(TopicAssociation, InferredImageTopicIsExcludedFromAdditional)
{
  BridgeTypeMapper mapper;
  const std::string imageTopic = "/sensor_test_robot_urdf_1/camera/image_raw";
  const std::string extraTopic = "/extra/scan";

  std::vector<GzTopicEntry> advertisedTopics{
    makeListedTopicWithoutType(imageTopic),
    makeEntry(extraTopic, "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll(
      "default",
      {makeSensor("robot", "base", "camera_sensor", "camera", imageTopic)},
      "",
      advertisedTopics);

  const auto additionalTopics = additionalTopicsAfterClaims(advertisedTopics, tree);
  ASSERT_EQ(additionalTopics.size(), 1u);
  EXPECT_EQ(additionalTopics[0], extraTopic);
}

TEST(TopicAssociation, InferredImuTopicIsExcludedFromAdditional)
{
  BridgeTypeMapper mapper;
  const std::string imuTopic = "/sensor_test_robot_urdf_1/imu/data_raw";
  const std::string extraTopic = "/extra/scan";

  std::vector<GzTopicEntry> advertisedTopics{
    makeListedTopicWithoutType(imuTopic),
    makeEntry(extraTopic, "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll(
      "default",
      {makeSensor("robot", "base", "imu_sensor", "imu", imuTopic)},
      "",
      advertisedTopics);

  const auto additionalTopics = additionalTopicsAfterClaims(advertisedTopics, tree);
  ASSERT_EQ(additionalTopics.size(), 1u);
  EXPECT_EQ(additionalTopics[0], extraTopic);
}

TEST(TopicAssociation, AttachedCameraInfoIsExcludedFromAdditional)
{
  BridgeTypeMapper mapper;
  const std::string imageTopic = "/sensor_test_robot_urdf_1/camera/image_raw";
  const std::string cameraInfoTopic = "/sensor_test_robot_urdf_1/camera/camera_info";
  const std::string extraTopic = "/extra/scan";

  std::vector<GzTopicEntry> advertisedTopics{
    makeListedTopicWithoutType(imageTopic),
    makeEntry(cameraInfoTopic, "gz.msgs.CameraInfo", mapper),
    makeEntry(extraTopic, "gz.msgs.LaserScan", mapper),
  };

  auto tree = TopicAssociation::matchAll(
      "default",
      {makeSensor("robot", "base", "camera_sensor", "camera", imageTopic)},
      "",
      advertisedTopics);

  const auto additionalTopics = additionalTopicsAfterClaims(advertisedTopics, tree);
  ASSERT_EQ(additionalTopics.size(), 1u);
  EXPECT_EQ(additionalTopics[0], extraTopic);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
