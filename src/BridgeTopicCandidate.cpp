#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

namespace gz_ros2_bridge_manager
{

const char *categoryName(AssociationCategory c)
{
  switch (c)
  {
    case AssociationCategory::Unsupported:   return "Unsupported";
    case AssociationCategory::Additional:    return "Additional";
    case AssociationCategory::EcmAssociated: return "EcmAssociated";
    default:                                 return "Unknown";
  }
}

}  // namespace gz_ros2_bridge_manager
