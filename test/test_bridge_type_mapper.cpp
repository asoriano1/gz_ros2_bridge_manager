#include <gtest/gtest.h>
#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"

using gz_ros2_bridge_manager::BridgeTypeMapper;

class BridgeTypeMapperTest : public ::testing::Test
{
protected:
  BridgeTypeMapper mapper;
};

TEST_F(BridgeTypeMapperTest, KnownTypesAreBridgeable)
{
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.LaserScan"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.Image"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.IMU"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.NavSat"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.PointCloudPacked"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.Magnetometer"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.FluidPressure"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.CameraInfo"));
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.Contacts"));
}

TEST_F(BridgeTypeMapperTest, UnknownTypeIsNotBridgeable)
{
  EXPECT_FALSE(mapper.isBridgeable("gz.msgs.UnknownFoo"));
  EXPECT_FALSE(mapper.isBridgeable(""));
  EXPECT_FALSE(mapper.isBridgeable("sensor_msgs/msg/LaserScan"));
}

TEST_F(BridgeTypeMapperTest, Ros2TypeMapping)
{
  EXPECT_EQ(mapper.ros2Type("gz.msgs.LaserScan"),        "sensor_msgs/msg/LaserScan");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.Image"),            "sensor_msgs/msg/Image");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.IMU"),              "sensor_msgs/msg/Imu");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.NavSat"),           "sensor_msgs/msg/NavSatFix");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.PointCloudPacked"), "sensor_msgs/msg/PointCloud2");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.Odometry"),         "nav_msgs/msg/Odometry");
  EXPECT_EQ(mapper.ros2Type("gz.msgs.Clock"),            "rosgraph_msgs/msg/Clock");
}

TEST_F(BridgeTypeMapperTest, UnknownTypeReturnsEmpty)
{
  EXPECT_EQ(mapper.ros2Type("gz.msgs.NotReal"), "");
  EXPECT_EQ(mapper.ros2Type(""),               "");
}

TEST_F(BridgeTypeMapperTest, BridgeSpecFormat)
{
  const std::string spec = mapper.bridgeSpec("/scan", "gz.msgs.LaserScan");
  EXPECT_EQ(spec, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
}

TEST_F(BridgeTypeMapperTest, BridgeSpecEmptyForUnknownType)
{
  EXPECT_EQ(mapper.bridgeSpec("/foo", "gz.msgs.NotReal"), "");
}

TEST_F(BridgeTypeMapperTest, KnownGzTypesNotEmpty)
{
  const auto types = mapper.knownGzTypes();
  EXPECT_GT(types.size(), 0u);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
