#include <gtest/gtest.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>

#include "gz_ros2_bridge_manager/EcmSensorExtractor.hh"

using namespace gz_ros2_bridge_manager;

TEST(EcmSensorExtractor, SensorTopicComponentPopulatesDeclaredTopic)
{
  gz::sim::EntityComponentManager ecm;

  const auto world = ecm.CreateEntity();
  ecm.CreateComponent(world, gz::sim::components::World());
  ecm.CreateComponent(world, gz::sim::components::Name("default"));

  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, gz::sim::components::Model());
  ecm.CreateComponent(model, gz::sim::components::Name("robot"));
  ecm.CreateComponent(model, gz::sim::components::ParentEntity(world));

  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, gz::sim::components::Name("camera_link"));
  ecm.CreateComponent(link, gz::sim::components::ParentEntity(model));

  const auto sensor = ecm.CreateEntity();
  ecm.CreateComponent(sensor, gz::sim::components::Sensor());
  ecm.CreateComponent(sensor, gz::sim::components::Camera());
  ecm.CreateComponent(sensor, gz::sim::components::Name("front_camera"));
  ecm.CreateComponent(sensor, gz::sim::components::ParentEntity(link));
  ecm.CreateComponent(sensor, gz::sim::components::SensorTopic("/robot/front_camera"));

  const auto sensors = EcmSensorExtractor::extract(ecm);
  ASSERT_EQ(sensors.size(), 1u);
  EXPECT_EQ(sensors[0].declaredTopic, "/robot/front_camera");
  EXPECT_EQ(sensors[0].sensorType, "camera");
  EXPECT_EQ(sensors[0].modelName, "robot");
  EXPECT_EQ(sensors[0].linkName, "camera_link");
  EXPECT_EQ(sensors[0].sensorName, "front_camera");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
