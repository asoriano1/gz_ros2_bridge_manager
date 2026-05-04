#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace gz_ros2_bridge_manager
{

using EntityId = uint64_t;

struct EcmSensorEntry
{
  EntityId modelEntity  = 0;
  std::string modelName;
  bool nestedModel = false;

  EntityId linkEntity   = 0;
  std::string linkName;

  EntityId sensorEntity = 0;
  std::string sensorName;
  std::string sensorType;

  std::string declaredTopic;
  std::string fallbackGazeboTopicPrefix;
};

struct EcmLinkEntry
{
  EntityId linkEntity = 0;
  std::string linkName;
};

struct EcmModelEntry
{
  EntityId modelEntity = 0;
  std::string modelName;
  bool nestedModel = false;
  std::vector<EcmLinkEntry> links;
  std::vector<EcmSensorEntry> sensors;
};

struct EcmWorldSnapshot
{
  std::string worldName;
  std::vector<EcmModelEntry> models;
  std::vector<EcmSensorEntry> sensors;
};

}  // namespace gz_ros2_bridge_manager
