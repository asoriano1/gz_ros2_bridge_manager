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

// ---- Multi-model tests ---------------------------------------------------
// In the accordion workflow the GUI concatenates candidates from all models
// (in discovery order) and calls BridgeCommandBuilder directly.

TEST(BridgeCommandBuilder, MultiModelCommandBuildsFromAllModels)
{
  // Simulate two models' candidates concatenated before command generation.
  std::vector<BridgeTopicCandidate> model_a{
    make("/model/robot_a/scan", true, true,
         "/model/robot_a/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
    make("/model/robot_a/imu",  true, true,
         "/model/robot_a/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"),
  };
  std::vector<BridgeTopicCandidate> model_b{
    make("/model/robot_b/scan", true, true,
         "/model/robot_b/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
  };

  std::vector<BridgeTopicCandidate> all;
  all.insert(all.end(), model_a.begin(), model_a.end());
  all.insert(all.end(), model_b.begin(), model_b.end());

  const auto specs = BridgeCommandBuilder::selectedSpecs(all);
  ASSERT_EQ(specs.size(), 3u);

  const std::string cmd = BridgeCommandBuilder::buildCommand(all);
  EXPECT_NE(cmd.find("/model/robot_a/scan"), std::string::npos);
  EXPECT_NE(cmd.find("/model/robot_a/imu"),  std::string::npos);
  EXPECT_NE(cmd.find("/model/robot_b/scan"), std::string::npos);
}

TEST(BridgeCommandBuilder, MultiModelCommandDeduplicatesAcrossModels)
{
  // Both models check the same global /clock topic — it must appear only once.
  const std::string clockSpec = "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock";

  std::vector<BridgeTopicCandidate> model_a{
    make("/clock", true, true, clockSpec),
    make("/model/robot_a/scan", true, true,
         "/model/robot_a/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
  };
  std::vector<BridgeTopicCandidate> model_b{
    make("/clock", true, true, clockSpec),  // same spec — duplicate
    make("/model/robot_b/scan", true, true,
         "/model/robot_b/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
  };

  std::vector<BridgeTopicCandidate> all;
  all.insert(all.end(), model_a.begin(), model_a.end());
  all.insert(all.end(), model_b.begin(), model_b.end());

  const auto specs = BridgeCommandBuilder::selectedSpecs(all);
  // /clock once, robot_a/scan, robot_b/scan = 3 unique specs.
  ASSERT_EQ(specs.size(), 3u);

  // Verify /clock appears exactly once in the command.
  const std::string cmd = BridgeCommandBuilder::buildCommand(all);
  const auto first  = cmd.find(clockSpec);
  ASSERT_NE(first, std::string::npos);
  EXPECT_EQ(cmd.find(clockSpec, first + 1), std::string::npos);
}

TEST(BridgeCommandBuilder, AdditionalTopicAppearsInCommand)
{
  // Additional bridgeable topics (not linked to any model) are appended last.
  std::vector<BridgeTopicCandidate> modelCands{
    make("/model/robot_a/scan", true, true,
         "/model/robot_a/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"),
  };
  std::vector<BridgeTopicCandidate> additionalCands{
    make("/clock", true, true, "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"),
  };

  std::vector<BridgeTopicCandidate> all;
  all.insert(all.end(), modelCands.begin(), modelCands.end());
  all.insert(all.end(), additionalCands.begin(), additionalCands.end());

  const auto specs = BridgeCommandBuilder::selectedSpecs(all);
  ASSERT_EQ(specs.size(), 2u);
  EXPECT_EQ(specs[0], "/model/robot_a/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan");
  EXPECT_EQ(specs[1], "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock");
}

TEST(BridgeCommandBuilder, IncludesInferredSensorTopicSpecWhenChecked)
{
  std::vector<BridgeTopicCandidate> cs{
    make("/sensor_test_robot_urdf_1/camera/image_raw", true, true,
         "/sensor_test_robot_urdf_1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"),
  };

  const auto specs = BridgeCommandBuilder::selectedSpecs(cs);
  ASSERT_EQ(specs.size(), 1u);
  EXPECT_EQ(specs[0],
            "/sensor_test_robot_urdf_1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image");

  const std::string cmd = BridgeCommandBuilder::buildCommand(cs);
  EXPECT_NE(cmd.find("/sensor_test_robot_urdf_1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"),
            std::string::npos);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
