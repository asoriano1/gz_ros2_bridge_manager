#pragma once

#include <memory>

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

namespace gz_ros2_bridge_manager
{

/// Owns the child process that runs `ros2 run ros_gz_bridge parameter_bridge`
/// (or its direct executable, when available).
///
/// Drives a small lifecycle state machine — Starting → Running → Stopping →
/// Stopped/Crashed/Exited — and exposes the running command, captured output
/// and a "restart required" flag back to the GUI via Qt signals.
///
/// The class also bundles a handful of static helpers (command building,
/// status formatting, output truncation) so they can be unit-tested without
/// instantiating a QProcess.
class BridgeProcessManager : public QObject
{
  Q_OBJECT

public:
  /// Lifecycle state of the managed bridge child process.
  enum class Status
  {
    NotRunning,        ///< No process has been started yet.
    Starting,          ///< QProcess::start() invoked, not yet running.
    Running,           ///< Child reported started.
    Stopping,          ///< Stop requested, waiting for graceful exit.
    RestartRequired,   ///< Selection changed while running; user must restart.
    Failed,            ///< Failed to start (e.g. executable not found).
    Crashed,           ///< Exited via signal or non-zero crash.
    Stopped,           ///< Clean exit triggered by stopBridge().
    Exited,            ///< Clean exit not triggered by the user.
  };
  Q_ENUM(Status)

  /// Fully resolved command ready to hand to QProcess::start().
  /// `direct=true` means we found the parameter_bridge binary directly and
  /// can skip the `ros2 run` wrapper.
  struct LaunchCommand
  {
    QString program;
    QStringList arguments;
    QString displayCommand;
    bool direct{false};
  };

  /// Hard cap on the captured stdout/stderr buffer to keep memory bounded.
  static constexpr int kDefaultMaxOutputChars = 16384;

  BridgeProcessManager();
  ~BridgeProcessManager() override;

  /// True while a child process exists and has not finished.
  bool bridgeRunning() const;
  /// True while a start/stop transition is in flight.
  bool bridgeBusy() const;
  /// Human-readable label derived from the current Status.
  QString bridgeStatusText() const;
  /// Captured stdout+stderr of the running child (truncated to kDefaultMaxOutputChars).
  QString bridgeOutput() const { return bridgeOutput_; }
  /// True when the desired spec list no longer matches what is running.
  bool bridgeRestartRequired() const { return bridgeRestartRequired_; }
  /// Display string of the command currently running, or "" if none.
  QString runningBridgeCommand() const { return runningBridgeCommand_; }
  /// True iff the latest stop was initiated by stopBridge() (vs. a crash).
  bool userStopRequested() const { return userStopRequested_; }

  /// Updates the desired list of bridge specs. If the bridge is running and
  /// the selection no longer matches, bridgeRestartRequired() becomes true.
  void setDesiredSpecs(const QStringList &specs);

  /// Asynchronously launches the bridge from the current desired specs.
  /// Returns false if no specs are selected or a launch is already in flight.
  bool runBridge();
  /// Requests a graceful shutdown of the running child (SIGINT, then SIGTERM).
  void stopBridge();
  /// Equivalent to stopBridge() followed by runBridge() once the child is gone.
  bool restartBridge();
  /// Clears the captured output buffer.
  void clearBridgeOutput();

  // ---- Stateless helpers (exposed for unit tests) ------------------------

  /// Composes the `ros2 run ros_gz_bridge parameter_bridge` argument list.
  static QStringList buildFallbackArguments(const QStringList &specs);
  /// Display string for `ros2 run ros_gz_bridge parameter_bridge ...`.
  static QString buildCommand(const QStringList &specs);
  /// Returns the absolute path to the `parameter_bridge` binary inside an
  /// installed `ros_gz_bridge` prefix, or "" if not found.
  static QString directExecutablePathForPrefix(const QString &prefix);
  /// Display string when launching parameter_bridge directly (no `ros2 run`).
  static QString buildDirectCommand(const QString &parameterBridgePath,
                                    const QStringList &specs);
  /// Picks the best launch strategy (direct vs. ros2-run) and assembles a
  /// ready-to-use LaunchCommand.
  static LaunchCommand buildLaunchCommand(const QString &parameterBridgePath,
                                          const QStringList &specs);
  /// User-facing label for a Status value.
  static QString statusToString(Status status);
  /// Decides whether the running command must be restarted because the
  /// selection diverged from what is on the wire.
  static bool requiresRestart(bool active,
                              const QString &runningCommand,
                              const QString &selectedCommand);
  /// Maps a QProcess exit notification + user-stop intent to the final Status.
  static Status finalStatusForExit(bool userStopRequested,
                                   int exitCode,
                                   QProcess::ExitStatus exitStatus);
  /// Appends `chunk` to `existing`, truncating from the front to keep the
  /// total bounded by `maxChars`.
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
