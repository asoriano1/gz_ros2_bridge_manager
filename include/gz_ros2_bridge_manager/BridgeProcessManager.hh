#pragma once

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
    Exited,
  };
  Q_ENUM(Status)

  static constexpr int kDefaultMaxOutputChars = 16384;

  BridgeProcessManager();
  ~BridgeProcessManager() override;

  bool bridgeRunning() const;
  bool bridgeBusy() const;
  QString bridgeStatusText() const;
  QString bridgeOutput() const { return bridgeOutput_; }
  bool bridgeRestartRequired() const { return bridgeRestartRequired_; }
  QString runningBridgeCommand() const { return runningBridgeCommand_; }

  void setDesiredSpecs(const QStringList &specs);

  bool runBridge();
  void stopBridge();
  bool restartBridge();
  void clearBridgeOutput();

  static QStringList buildArguments(const QStringList &specs);
  static QString buildCommand(const QStringList &specs);
  static QString statusToString(Status status);
  static bool requiresRestart(bool active,
                              const QString &runningCommand,
                              const QString &selectedCommand);
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
  void onStopTimeout();

private:
  static constexpr int kStopTimeoutMs = 1500;

  void requestStop(bool restartAfterStop);
  bool startDesiredBridge();
  void setStatus(Status status);
  void setRestartRequired(bool required);
  void setRunningBridgeCommand(const QString &command);
  void clearRunningProcessState();
  void updateRestartRequired();
  void appendOutput(const QString &text);

  QProcess process_;
  QTimer stopTimer_;

  QStringList desiredSpecs_;
  QString desiredCommand_;
  QString runningBridgeCommand_;
  QString bridgeOutput_;

  Status status_{Status::NotRunning};
  bool bridgeRestartRequired_{false};
  bool restartPending_{false};
  bool failedToStart_{false};
};

}  // namespace gz_ros2_bridge_manager
