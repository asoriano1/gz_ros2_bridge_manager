#pragma once

#include <memory>

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

namespace gz_ros2_bridge_manager
{

class BridgeProcessManager : public QObject
{
  Q_OBJECT

public:
  enum class Status
  {
    NotRunning,
    Starting,
    Running,
    Stopping,
    RestartRequired,
    Failed,
    Crashed,
    Stopped,
    Exited,
  };
  Q_ENUM(Status)

  struct LaunchCommand
  {
    QString program;
    QStringList arguments;
    QString displayCommand;
    bool direct{false};
  };

  static constexpr int kDefaultMaxOutputChars = 16384;

  BridgeProcessManager();
  ~BridgeProcessManager() override;

  bool bridgeRunning() const;
  bool bridgeBusy() const;
  QString bridgeStatusText() const;
  QString bridgeOutput() const { return bridgeOutput_; }
  bool bridgeRestartRequired() const { return bridgeRestartRequired_; }
  QString runningBridgeCommand() const { return runningBridgeCommand_; }
  bool userStopRequested() const { return userStopRequested_; }

  void setDesiredSpecs(const QStringList &specs);

  bool runBridge();
  void stopBridge();
  bool restartBridge();
  void clearBridgeOutput();

  static QStringList buildFallbackArguments(const QStringList &specs);
  static QString buildCommand(const QStringList &specs);
  static QString directExecutablePathForPrefix(const QString &prefix);
  static QString buildDirectCommand(const QString &parameterBridgePath,
                                    const QStringList &specs);
  static LaunchCommand buildLaunchCommand(const QString &parameterBridgePath,
                                          const QStringList &specs);
  static QString statusToString(Status status);
  static bool requiresRestart(bool active,
                              const QString &runningCommand,
                              const QString &selectedCommand);
  static Status finalStatusForExit(bool userStopRequested,
                                   int exitCode,
                                   QProcess::ExitStatus exitStatus);
  static QString appendBoundedOutput(const QString &existing,
                                     const QString &chunk,
                                     int maxChars = kDefaultMaxOutputChars);

signals:
  void bridgeRunningChanged();
  void bridgeBusyChanged();
  void bridgeStatusTextChanged();
  void bridgeOutputChanged();
  void bridgeRestartRequiredChanged();
  void runningBridgeCommandChanged();

private slots:
  void onStarted();
  void onReadyReadStandardOutput();
  void onReadyReadStandardError();
  void onProcessError(QProcess::ProcessError error);
  void onFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onResolverFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void onResolverError(QProcess::ProcessError error);
  void onStopTimeout();

private:
  static constexpr int kStopTimeoutMs = 1500;

  void requestStop(bool restartAfterStop);
  bool startDesiredBridge();
  bool startBridgeWithLaunchCommand(const LaunchCommand &launchCommand);
  void resolveExecutableAndStart();
  void setStatus(Status status);
  void setRestartRequired(bool required);
  void setRunningBridgeCommand(const QString &command);
  void clearRunningProcessState();
  void updateRestartRequired();
  void appendOutput(const QString &text);
  void failStart(const QString &message);
  bool sendSignalToManagedProcessGroup(int signalNumber);
  void stopManagedProcessGroupBlocking();

  std::unique_ptr<QProcess> process_;
  QProcess resolverProcess_;
  QTimer stopTimer_;

  QStringList desiredSpecs_;
  QString desiredCommandKey_;
  QString runningCommandKey_;
  QString runningBridgeCommand_;
  QString bridgeOutput_;
  QString resolvedParameterBridgePath_;

  Status status_{Status::NotRunning};
  bool bridgeRestartRequired_{false};
  bool restartPending_{false};
  bool failedToStart_{false};
  bool userStopRequested_{false};
  bool resolvingExecutable_{false};
  qint64 managedProcessId_{0};
  qint64 managedProcessGroupId_{0};
};

}  // namespace gz_ros2_bridge_manager
