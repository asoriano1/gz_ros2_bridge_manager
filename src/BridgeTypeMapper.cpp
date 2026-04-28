#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"

namespace gz_ros2_bridge_manager
{

BridgeTypeMapper::BridgeTypeMapper()
{
  // Sensor types
  mappings_["gz.msgs.Image"]            = "sensor_msgs/msg/Image";
  mappings_["gz.msgs.CameraInfo"]       = "sensor_msgs/msg/CameraInfo";
  mappings_["gz.msgs.LaserScan"]        = "sensor_msgs/msg/LaserScan";
  mappings_["gz.msgs.IMU"]              = "sensor_msgs/msg/Imu";
  mappings_["gz.msgs.NavSat"]           = "sensor_msgs/msg/NavSatFix";
  mappings_["gz.msgs.PointCloudPacked"] = "sensor_msgs/msg/PointCloud2";
  mappings_["gz.msgs.Contacts"]         = "ros_gz_interfaces/msg/Contacts";
  mappings_["gz.msgs.Magnetometer"]     = "sensor_msgs/msg/MagneticField";
  mappings_["gz.msgs.FluidPressure"]    = "sensor_msgs/msg/FluidPressure";
  mappings_["gz.msgs.BatteryState"]     = "sensor_msgs/msg/BatteryState";

  // Geometry / TF
  mappings_["gz.msgs.Pose"]             = "geometry_msgs/msg/Pose";
  mappings_["gz.msgs.PoseWithCovariance"] = "geometry_msgs/msg/PoseWithCovariance";
  mappings_["gz.msgs.Pose_V"]           = "tf2_msgs/msg/TFMessage";
  mappings_["gz.msgs.Twist"]            = "geometry_msgs/msg/Twist";
  mappings_["gz.msgs.TwistWithCovariance"] = "geometry_msgs/msg/TwistWithCovariance";
  mappings_["gz.msgs.Odometry"]         = "nav_msgs/msg/Odometry";
  mappings_["gz.msgs.OdometryWithCovariance"] = "ros_gz_interfaces/msg/OdometryWithCovariance";

  // Simulation state
  mappings_["gz.msgs.Clock"]            = "rosgraph_msgs/msg/Clock";
  mappings_["gz.msgs.EntityWrench"]     = "ros_gz_interfaces/msg/EntityWrench";
  mappings_["gz.msgs.Float"]            = "std_msgs/msg/Float32";
  mappings_["gz.msgs.Double"]           = "std_msgs/msg/Float64";
  mappings_["gz.msgs.Int32"]            = "std_msgs/msg/Int32";
  mappings_["gz.msgs.StringMsg"]        = "std_msgs/msg/String";
  mappings_["gz.msgs.Boolean"]          = "std_msgs/msg/Bool";
}

bool BridgeTypeMapper::isBridgeable(const std::string &gzMsgType) const
{
  return mappings_.count(gzMsgType) > 0;
}

std::string BridgeTypeMapper::ros2Type(const std::string &gzMsgType) const
{
  auto it = mappings_.find(gzMsgType);
  return (it != mappings_.end()) ? it->second : "";
}

std::string BridgeTypeMapper::bridgeSpec(const std::string &topic,
                                          const std::string &gzMsgType) const
{
  const std::string ros2 = ros2Type(gzMsgType);
  if (ros2.empty())
    return "";
  return topic + "@" + ros2 + "@" + gzMsgType;
}

std::vector<std::string> BridgeTypeMapper::knownGzTypes() const
{
  std::vector<std::string> types;
  types.reserve(mappings_.size());
  for (const auto &kv : mappings_)
    types.push_back(kv.first);
  return types;
}

}  // namespace gz_ros2_bridge_manager
