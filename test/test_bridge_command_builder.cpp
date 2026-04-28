#include <gtest/gtest.h>

#include "gz_ros2_bridge_manager/BridgeCommandBuilder.hh"
#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

BridgeTopicCandidate make(const std::string &topic, bool bridgeable,
                         bool checked, const std::string &spec = "")
{
  BridgeTopicCandidate c;
  c.gzTopic    = topic;
  c.bridgeable = bridgeable;
  c.checked    = checked;
  c.bridgeSpec = spec.empty() && bridgeable
                   ? topic + "@sensor_msgs/msg/Foo@gz.msgs.Foo"
                   : spec;
  return c;
}

}  // namespace

TEST(BridgeCommandBuilder, EmptyWhenNothingChecked)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/a", true, false),
    make("/b", false, false),
  };
  EXPECT_EQ(BridgeCommandBuilder::buildCommand(cs),       "");
  EXPECT_EQ(BridgeCommandBuilder::buildCommandWrapped(cs), "");
  EXPECT_TRUE(BridgeCommandBuilder::selectedSpecs(cs).empty());
}

TEST(BridgeCommandBuilder, IncludesOnlyCheckedAndBridgeable)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/scan", true,  true,  "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
    make("/imu",  true,  false, "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"),
    make("/foo",  false, true,  ""),
  };
  const auto specs = BridgeCommandBuilder::selectedSpecs(cs);
  ASSERT_EQ(specs.size(), 1u);
  EXPECT_EQ(specs[0], "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
}

TEST(BridgeCommandBuilder, DeduplicatesIdenticalSpecs)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/scan", true, true, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
    make("/scan", true, true, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
    make("/imu",  true, true, "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"),
  };
  const auto specs = BridgeCommandBuilder::selectedSpecs(cs);
  ASSERT_EQ(specs.size(), 2u);
  EXPECT_EQ(specs[0], "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
  EXPECT_EQ(specs[1], "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU");
}

TEST(BridgeCommandBuilder, FormatsSingleLineCommand)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/scan", true, true, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
    make("/imu",  true, true, "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"),
  };
  const std::string cmd = BridgeCommandBuilder::buildCommand(cs);
  EXPECT_EQ(cmd,
            "ros2 run ros_gz_bridge parameter_bridge "
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan "
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU");
  // Single-line form: no embedded newlines.
  EXPECT_EQ(cmd.find('\n'), std::string::npos);
}

TEST(BridgeCommandBuilder, WrappedCommandHasContinuations)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/a", true, true, "/a@A@a"),
    make("/b", true, true, "/b@B@b"),
  };
  const std::string wrapped = BridgeCommandBuilder::buildCommandWrapped(cs);
  EXPECT_NE(wrapped.find("\\\n  "), std::string::npos);
  EXPECT_NE(wrapped.find("/a@A@a"), std::string::npos);
  EXPECT_NE(wrapped.find("/b@B@b"), std::string::npos);
}

TEST(BridgeCommandBuilder, SkipsCheckedButUnsupported)
{
  // A checked, NON-bridgeable candidate must not appear in the command.
  std::vector<BridgeTopicCandidate> cs{
    make("/weird", false, true, ""),
    make("/scan",  true,  true, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
  };
  const std::string cmd = BridgeCommandBuilder::buildCommand(cs);
  EXPECT_EQ(cmd.find("/weird"), std::string::npos);
  EXPECT_NE(cmd.find("/scan"),  std::string::npos);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
