#include <gtest/gtest.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>

#include "gz_ros2_bridge_manager/EcmSensorExtractor.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

struct TestHierarchy
{
  gz::sim::Entity world{gz::sim::kNullEntity};
  gz::sim::Entity model{gz::sim::kNullEntity};
  gz::sim::Entity link{gz::sim::kNullEntity};
};

TestHierarchy makeWorldModelLink(gz::sim::EntityComponentManager &ecm)
{
  TestHierarchy h;

  h.world = ecm.CreateEntity();
  ecm.CreateComponent(h.world, gz::sim::components::World());
  ecm.CreateComponent(h.world, gz::sim::components::Name("default"));

  h.model = ecm.CreateEntity();
  ecm.CreateComponent(h.model, gz::sim::components::Model());
  ecm.CreateComponent(h.model, gz::sim::components::Name("robot"));
  ecm.CreateComponent(h.model, gz::sim::components::ParentEntity(h.world));

  h.link = ecm.CreateEntity();
  ecm.CreateComponent(h.link, gz::sim::components::Link());
  ecm.CreateComponent(h.link, gz::sim::components::Name("camera_link"));
  ecm.CreateComponent(h.link, gz::sim::components::ParentEntity(h.model));

  return h;
}

}  // namespace

TEST(EcmSensorExtractor, SensorUnderLinkUsesSensorTopicAndOwningModel)
{
  gz::sim::EntityComponentManager ecm;
  const auto h = makeWorldModelLink(ecm);

  const auto sensor = ecm.CreateEntity();
  ecm.CreateComponent(sensor, gz::sim::components::Sensor());
  ecm.CreateComponent(sensor, gz::sim::components::Camera());
  ecm.CreateComponent(sensor, gz::sim::components::Name("front_camera"));
  ecm.CreateComponent(sensor, gz::sim::components::ParentEntity(h.link));
  ecm.CreateComponent(sensor, gz::sim::components::SensorTopic("/robot/front_camera"));

  const auto sensors = EcmSensorExtractor::extract(ecm);
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0].declaredTopic, "/robot/front_camera");
  EXPECT_EQ(sensors[0].sensorType, "camera");
  EXPECT_EQ(sensors[0].modelName, "robot");
  EXPECT_EQ(sensors[0].linkName, "camera_link");
  EXPECT_EQ(sensors[0].sensorName, "front_camera");
}

TEST(EcmSensorExtractor, SensorDirectlyUnderModelUsesSensorTopicAndOwningModel)
{
  gz::sim::EntityComponentManager ecm;
  const auto h = makeWorldModelLink(ecm);

  const auto sensor = ecm.CreateEntity();
  ecm.CreateComponent(sensor, gz::sim::components::Sensor());
  ecm.CreateComponent(sensor, gz::sim::components::Imu());
  ecm.CreateComponent(sensor, gz::sim::components::Name("imu_sensor"));
  ecm.CreateComponent(sensor, gz::sim::components::ParentEntity(h.model));
  ecm.CreateComponent(sensor, gz::sim::components::SensorTopic("/robot/imu/data_raw"));

  const auto sensors = EcmSensorExtractor::extract(ecm);
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0].declaredTopic, "/robot/imu/data_raw");
  EXPECT_EQ(sensors[0].sensorType, "imu");
  EXPECT_EQ(sensors[0].modelName, "robot");
  EXPECT_TRUE(sensors[0].linkName.empty());
  EXPECT_EQ(sensors[0].sensorName, "imu_sensor");
}

TEST(EcmSensorExtractor, ExtractsAllSensorEntitiesAcrossSupportedParentPatterns)
{
  gz::sim::EntityComponentManager ecm;
  const auto h = makeWorldModelLink(ecm);

  const auto camera = ecm.CreateEntity();
  ecm.CreateComponent(camera, gz::sim::components::Sensor());
  ecm.CreateComponent(camera, gz::sim::components::Camera());
  ecm.CreateComponent(camera, gz::sim::components::Name("front_camera"));
  ecm.CreateComponent(camera, gz::sim::components::ParentEntity(h.link));
  ecm.CreateComponent(camera, gz::sim::components::SensorTopic("/robot/front_camera"));

  const auto imu = ecm.CreateEntity();
  ecm.CreateComponent(imu, gz::sim::components::Sensor());
  ecm.CreateComponent(imu, gz::sim::components::Imu());
  ecm.CreateComponent(imu, gz::sim::components::Name("imu_sensor"));
  ecm.CreateComponent(imu, gz::sim::components::ParentEntity(h.model));
  ecm.CreateComponent(imu, gz::sim::components::SensorTopic("/robot/imu/data_raw"));

  const auto lidar = ecm.CreateEntity();
  ecm.CreateComponent(lidar, gz::sim::components::Sensor());
  ecm.CreateComponent(lidar, gz::sim::components::GpuLidar());
  ecm.CreateComponent(lidar, gz::sim::components::Name("lidar"));
  ecm.CreateComponent(lidar, gz::sim::components::ParentEntity(h.model));
  ecm.CreateComponent(lidar, gz::sim::components::SensorTopic("/robot/lidar/scan"));

  const auto sensors = EcmSensorExtractor::extract(ecm);
  ASSERT_EQ(sensors.size(), 3u);

  size_t directModelSensors = 0;
  size_t linkSensors = 0;
  for (const auto &sensor : sensors)
  {
    EXPECT_EQ(sensor.modelName, "robot");
    EXPECT_FALSE(sensor.declaredTopic.empty());
    if (sensor.linkName.empty())
      ++directModelSensors;
    else if (sensor.linkName == "camera_link")
      ++linkSensors;
  }

  EXPECT_EQ(directModelSensors, 2u);
  EXPECT_EQ(linkSensors, 1u);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
