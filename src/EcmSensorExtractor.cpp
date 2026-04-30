#include "gz_ros2_bridge_manager/EcmSensorExtractor.hh"

#include <functional>

#include <gz/common/Console.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/AirPressureSensor.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/NavSat.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/components/Sensor.hh>  // also defines SensorTopic
#include <gz/sim/components/World.hh>

namespace gz_ros2_bridge_manager
{

namespace
{

// Boost-style hash combine.
inline void hashCombine(size_t &seed, size_t v)
{
  seed ^= v + 0x9e3779b9u + (seed << 6) + (seed >> 2);
}

std::string detectSensorType(const gz::sim::EntityComponentManager &ecm,
                              gz::sim::Entity entity)
{
  if (ecm.Component<gz::sim::components::Camera>(entity))           return "camera";
  if (ecm.Component<gz::sim::components::GpuLidar>(entity))         return "gpu_lidar";
  if (ecm.Component<gz::sim::components::Lidar>(entity))            return "lidar";
  if (ecm.Component<gz::sim::components::Imu>(entity))              return "imu";
  if (ecm.Component<gz::sim::components::DepthCamera>(entity))      return "depth_camera";
  if (ecm.Component<gz::sim::components::RgbdCamera>(entity))       return "rgbd_camera";
  if (ecm.Component<gz::sim::components::ForceTorque>(entity))      return "force_torque";
  if (ecm.Component<gz::sim::components::Magnetometer>(entity))     return "magnetometer";
  if (ecm.Component<gz::sim::components::AirPressureSensor>(entity)) return "air_pressure";
  if (ecm.Component<gz::sim::components::NavSat>(entity))           return "navsat";
  return "unknown";
}

std::string getWorldName(const gz::sim::EntityComponentManager &ecm)
{
  std::string name;
  ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
    [&](const gz::sim::Entity &,
        const gz::sim::components::World *,
        const gz::sim::components::Name *nameCmp) -> bool
    {
      name = nameCmp->Data();
      return false;  // stop after first world
    });
  return name;
}

}  // namespace

// ---------------------------------------------------------------------------

size_t EcmSensorExtractor::computeFingerprint(
    const gz::sim::EntityComponentManager &ecm)
{
  size_t fp = 0;
  ecm.Each<gz::sim::components::Sensor>(
    [&](const gz::sim::Entity &sensorEnt,
        const gz::sim::components::Sensor *) -> bool
    {
      hashCombine(fp, std::hash<uint64_t>{}(static_cast<uint64_t>(sensorEnt)));

      const auto *topicCmp =
          ecm.Component<gz::sim::components::SensorTopic>(sensorEnt);
      const std::string &topicVal = topicCmp ? topicCmp->Data() : std::string{};
      hashCombine(fp, std::hash<std::string>{}(topicVal));

      return true;
    });
  return fp;
}

// ---------------------------------------------------------------------------

std::vector<EcmSensorEntry> EcmSensorExtractor::extract(
    const gz::sim::EntityComponentManager &ecm)
{
  const std::string worldName = getWorldName(ecm);

  std::vector<EcmSensorEntry> result;

  ecm.Each<gz::sim::components::Sensor,
            gz::sim::components::Name,
            gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &sensorEnt,
        const gz::sim::components::Sensor *,
        const gz::sim::components::Name *nameCmp,
        const gz::sim::components::ParentEntity *parentCmp) -> bool
    {
      EcmSensorEntry e;
      e.sensorEntity = static_cast<EntityId>(sensorEnt);
      e.sensorName   = nameCmp->Data();
      e.sensorType   = detectSensorType(ecm, sensorEnt);

      const auto *topicCmp =
          ecm.Component<gz::sim::components::SensorTopic>(sensorEnt);
      // Gazebo's Entity Tree / Component Inspector surfaces this exact
      // components::SensorTopic value as "Sensor Topic" for the sensor entity.
      if (topicCmp)
        e.declaredTopic = topicCmp->Data();

      gz::sim::Entity parentEnt = parentCmp->Data();
      gz::sim::Entity currentEnt = parentEnt;

      while (currentEnt != gz::sim::kNullEntity)
      {
        if (e.linkEntity == 0 &&
            ecm.Component<gz::sim::components::Link>(currentEnt) != nullptr)
        {
          e.linkEntity = static_cast<EntityId>(currentEnt);
          const auto *linkNameCmp =
              ecm.Component<gz::sim::components::Name>(currentEnt);
          if (linkNameCmp)
            e.linkName = linkNameCmp->Data();
        }

        if (ecm.Component<gz::sim::components::Model>(currentEnt) != nullptr)
        {
          e.modelEntity = static_cast<EntityId>(currentEnt);
          const auto *modelNameCmp =
              ecm.Component<gz::sim::components::Name>(currentEnt);
          if (modelNameCmp)
            e.modelName = modelNameCmp->Data();

          const auto *modelParentCmp =
              ecm.Component<gz::sim::components::ParentEntity>(currentEnt);
          if (modelParentCmp)
          {
            gz::sim::Entity grandParent = modelParentCmp->Data();
            e.nestedModel =
                ecm.Component<gz::sim::components::Model>(grandParent) != nullptr;
          }
          break;
        }

        const auto *ancestorParentCmp =
            ecm.Component<gz::sim::components::ParentEntity>(currentEnt);
        if (!ancestorParentCmp)
          break;
        currentEnt = ancestorParentCmp->Data();
      }

      // Pre-compute the Gazebo standard topic prefix for this sensor.
      if (!worldName.empty() && !e.modelName.empty() && !e.sensorName.empty())
      {
        if (!e.linkName.empty())
        {
          e.fallbackGazeboTopicPrefix =
              "/world/" + worldName +
              "/model/" + e.modelName +
              "/link/"  + e.linkName  +
              "/sensor/" + e.sensorName;
        }
        else
        {
          e.fallbackGazeboTopicPrefix =
              "/world/" + worldName +
              "/model/" + e.modelName +
              "/sensor/" + e.sensorName;
        }
      }

      if (e.declaredTopic.empty() &&
          (e.sensorType == "lidar" || e.sensorType == "gpu_lidar"))
      {
        gzdbg << "[BridgeManager] ECM lidar sensor missing SensorTopic: "
              << "sensor=" << e.sensorName
              << ", type=" << e.sensorType
              << ", model=" << e.modelName
              << ", link=" << (e.linkName.empty() ? "<direct-model-child>" : e.linkName)
              << ", sensorEntity=" << sensorEnt
              << ", components=Sensor,"
              << (e.sensorType == "gpu_lidar" ? "GpuLidar" : "Lidar")
              << ",ParentEntity,Name\n";
      }

      result.push_back(std::move(e));
      return true;
    });

  return result;
}

// ---------------------------------------------------------------------------

size_t EcmSensorExtractor::countModels(
    const gz::sim::EntityComponentManager &ecm)
{
  size_t count = 0;
  ecm.Each<gz::sim::components::Model>(
    [&](const gz::sim::Entity &,
        const gz::sim::components::Model *) -> bool
    {
      ++count;
      return true;
    });
  return count;
}

}  // namespace gz_ros2_bridge_manager
