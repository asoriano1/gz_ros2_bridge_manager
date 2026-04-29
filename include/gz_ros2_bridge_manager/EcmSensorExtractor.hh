#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>

#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"

namespace gz_ros2_bridge_manager
{

/// Extracts sensor hierarchy from the Gazebo EntityComponentManager.
///
/// All gz-sim8 dependencies live in EcmSensorExtractor.cpp; this header is
/// included by Ros2BridgeManagerGui.hh which is also processed by Qt's MOC,
/// so it must remain gz-sim-free.
class EcmSensorExtractor
{
public:
  /// Hash of (sensor entity IDs + SensorTopic values).
  /// Changes when sensors are added/removed or SensorTopic values populate.
  static size_t computeFingerprint(const gz::sim::EntityComponentManager &ecm);

  /// Walk the ECM and produce one EcmSensorEntry per sensor entity.
  /// The world name is queried directly from the ECM (World + Name components).
  /// Each entry's fallbackGazeboTopicPrefix is set to
  ///   /world/<world>/model/<model>/link/<link>/sensor/<sensor>
  static std::vector<EcmSensorEntry> extract(
      const gz::sim::EntityComponentManager &ecm);

  /// Count all entities that carry a Model component.
  static size_t countModels(const gz::sim::EntityComponentManager &ecm);
};

}  // namespace gz_ros2_bridge_manager
