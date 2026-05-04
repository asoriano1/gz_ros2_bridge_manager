#pragma once

#include <cstddef>
#include <gz/sim/EntityComponentManager.hh>

#include "gz_ros2_bridge_manager/EcmTypes.hh"

namespace gz_ros2_bridge_manager
{

/// Extracts ECM-side world / model / link / sensor state from gz-sim.
///
/// All gz-sim8 dependencies live in EcmDiscovery.cpp; this header is included
/// by Ros2BridgeManagerGui.hh which is also processed by Qt's MOC, so it must
/// remain gz-sim-free.
class EcmDiscovery
{
public:
  /// Hash of world/model/link/sensor entities and SensorTopic values.
  /// Changes when ECM discovery-relevant state changes.
  static size_t computeFingerprint(const gz::sim::EntityComponentManager &ecm);

  /// Walk the ECM and produce a snapshot of the currently discovered world,
  /// models, links and sensors. Each sensor's fallbackGazeboTopicPrefix is set
  /// to:
  ///   /world/<world>/model/<model>/link/<link>/sensor/<sensor>
  static EcmWorldSnapshot extract(
      const gz::sim::EntityComponentManager &ecm);
};

}  // namespace gz_ros2_bridge_manager
