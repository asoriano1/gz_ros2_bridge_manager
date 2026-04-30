#include <gtest/gtest.h>

#include "gz_ros2_bridge_manager/BridgeProcessManager.hh"

using namespace gz_ros2_bridge_manager;

TEST(BridgeProcessManager, BuildsDirectParameterBridgeLaunchCommand)
{
  const QStringList specs{
      QStringLiteral("/cam@sensor_msgs/msg/Image@gz.msgs.Image"),
      QStringLiteral("/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"),
  };

  const auto launch = BridgeProcessManager::buildLaunchCommand(
      QStringLiteral("/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"),
      specs);

  EXPECT_TRUE(launch.direct);
  EXPECT_EQ(launch.program,
            QStringLiteral("/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"));
  EXPECT_EQ(launch.arguments, specs);
  EXPECT_EQ(
      launch.displayCommand,
      QStringLiteral("/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge "
                     "/cam@sensor_msgs/msg/Image@gz.msgs.Image "
                     "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"));
}

TEST(BridgeProcessManager, FallsBackToRos2RunWhenExecutableResolutionFails)
{
  const QStringList specs{
      QStringLiteral("/cam@sensor_msgs/msg/Image@gz.msgs.Image"),
  };

  const auto launch = BridgeProcessManager::buildLaunchCommand(QString{}, specs);

  EXPECT_FALSE(launch.direct);
  EXPECT_EQ(launch.program, QStringLiteral("ros2"));
  ASSERT_EQ(launch.arguments.size(), 4);
  EXPECT_EQ(launch.arguments[0], QStringLiteral("run"));
  EXPECT_EQ(launch.arguments[1], QStringLiteral("ros_gz_bridge"));
  EXPECT_EQ(launch.arguments[2], QStringLiteral("parameter_bridge"));
  EXPECT_EQ(launch.arguments[3], specs[0]);
  EXPECT_EQ(launch.displayCommand, BridgeProcessManager::buildCommand(specs));
}

TEST(BridgeProcessManager, EmptySpecsAreRefused)
{
  BridgeProcessManager manager;

  manager.setDesiredSpecs(QStringList{});

  EXPECT_FALSE(manager.runBridge());
  EXPECT_EQ(manager.bridgeStatusText(), QStringLiteral("Failed"));
  EXPECT_FALSE(manager.bridgeRunning());
  EXPECT_FALSE(manager.bridgeBusy());
  EXPECT_NE(manager.bridgeOutput().indexOf(QStringLiteral("no topics selected")),
            -1);
}

TEST(BridgeProcessManager, RestartRequiredWhenCommandChangesWhileRunning)
{
  const QString running = QStringLiteral(
      "ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU");
  const QString selected = QStringLiteral(
      "ros2 run ros_gz_bridge parameter_bridge /cam@sensor_msgs/msg/Image@gz.msgs.Image");

  EXPECT_TRUE(
      BridgeProcessManager::requiresRestart(true, running, selected));
}

TEST(BridgeProcessManager, SameCommandDoesNotRequireRestart)
{
  const QString command = QStringLiteral(
      "ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU");

  EXPECT_FALSE(
      BridgeProcessManager::requiresRestart(true, command, command));
  EXPECT_FALSE(
      BridgeProcessManager::requiresRestart(false, command, QStringLiteral("")));
}

TEST(BridgeProcessManager, StatusToStringCoversPublicStates)
{
  EXPECT_EQ(BridgeProcessManager::statusToString(
                BridgeProcessManager::Status::NotRunning),
            QStringLiteral("Not running"));
  EXPECT_EQ(BridgeProcessManager::statusToString(
                BridgeProcessManager::Status::Starting),
            QStringLiteral("Starting"));
  EXPECT_EQ(BridgeProcessManager::statusToString(
                BridgeProcessManager::Status::RestartRequired),
            QStringLiteral("Restart required"));
  EXPECT_EQ(BridgeProcessManager::statusToString(
                BridgeProcessManager::Status::Crashed),
            QStringLiteral("Crashed"));
  EXPECT_EQ(BridgeProcessManager::statusToString(
                BridgeProcessManager::Status::Stopped),
            QStringLiteral("Stopped"));
}

TEST(BridgeProcessManager, OutputBufferTruncatesOldestText)
{
  const QString bounded = BridgeProcessManager::appendBoundedOutput(
      QStringLiteral("abcdef"),
      QStringLiteral("ghijkl"),
      8);

  EXPECT_EQ(bounded, QStringLiteral("efghijkl"));
}

TEST(BridgeProcessManager, UserRequestedSignalExitMapsToStopped)
{
  EXPECT_EQ(
      BridgeProcessManager::finalStatusForExit(
          true, 130, QProcess::CrashExit),
      BridgeProcessManager::Status::Stopped);
}

TEST(BridgeProcessManager, UnexpectedCrashMapsToCrashed)
{
  EXPECT_EQ(
      BridgeProcessManager::finalStatusForExit(
          false, 1, QProcess::CrashExit),
      BridgeProcessManager::Status::Crashed);
}

TEST(BridgeProcessManager, StopRequestSetsUserStopRequestedFlag)
{
  BridgeProcessManager manager;
  manager.stopBridge();
  EXPECT_TRUE(manager.userStopRequested());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
