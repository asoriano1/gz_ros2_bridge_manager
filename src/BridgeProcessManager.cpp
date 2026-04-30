#include "gz_ros2_bridge_manager/BridgeProcessManager.hh"

#include <QStringBuilder>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>

#ifdef Q_OS_LINUX
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace gz_ros2_bridge_manager
{

namespace
{

class BridgeChildProcess : public QProcess
{
public:
  using QProcess::QProcess;

protected:
  void setupChildProcess() override
  {
    QProcess::setupChildProcess();

#ifdef Q_OS_LINUX
    if (::setsid() < 0)
      ::setpgid(0, 0);
#endif
  }
};

}  // namespace

BridgeProcessManager::BridgeProcessManager()
{
  process_ = std::make_unique<BridgeChildProcess>();
  process_->setProcessChannelMode(QProcess::SeparateChannels);
  resolverProcess_.setProcessChannelMode(QProcess::SeparateChannels);

  stopTimer_.setSingleShot(true);
  stopTimer_.setInterval(kStopTimeoutMs);

  connect(process_.get(), &QProcess::started,
          this, &BridgeProcessManager::onStarted);
  connect(process_.get(), &QProcess::readyReadStandardOutput,
          this, &BridgeProcessManager::onReadyReadStandardOutput);
  connect(process_.get(), &QProcess::readyReadStandardError,
          this, &BridgeProcessManager::onReadyReadStandardError);
  connect(process_.get(), &QProcess::errorOccurred,
          this, &BridgeProcessManager::onProcessError);
  connect(process_.get(),
          qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          this,
          &BridgeProcessManager::onFinished);
  connect(&resolverProcess_,
          qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          this,
          &BridgeProcessManager::onResolverFinished);
  connect(&resolverProcess_, &QProcess::errorOccurred,
          this, &BridgeProcessManager::onResolverError);
  connect(&stopTimer_, &QTimer::timeout,
          this, &BridgeProcessManager::onStopTimeout);
}

BridgeProcessManager::~BridgeProcessManager()
{
  restartPending_ = false;
  stopTimer_.stop();

  if (resolverProcess_.state() != QProcess::NotRunning)
  {
    resolverProcess_.blockSignals(true);
    resolverProcess_.kill();
    resolverProcess_.waitForFinished(300);
  }

  stopManagedProcessGroupBlocking();
}

bool BridgeProcessManager::bridgeRunning() const
{
  switch (status_)
  {
    case Status::Starting:
    case Status::Running:
    case Status::Stopping:
    case Status::RestartRequired:
      return true;
    case Status::NotRunning:
    case Status::Failed:
    case Status::Crashed:
    case Status::Stopped:
    case Status::Exited:
      return false;
  }
  return false;
}

bool BridgeProcessManager::bridgeBusy() const
{
  return status_ == Status::Starting || status_ == Status::Stopping;
}

QString BridgeProcessManager::bridgeStatusText() const
{
  return statusToString(status_);
}

void BridgeProcessManager::setDesiredSpecs(const QStringList &specs)
{
  desiredSpecs_ = specs;
  desiredCommandKey_ = buildCommand(desiredSpecs_);
  updateRestartRequired();
}

bool BridgeProcessManager::runBridge()
{
  restartPending_ = false;

  if (bridgeRunning())
    return false;

  return startDesiredBridge();
}

void BridgeProcessManager::stopBridge()
{
  requestStop(false);
}

bool BridgeProcessManager::restartBridge()
{
  if (!bridgeRunning())
  {
    restartPending_ = false;
    return startDesiredBridge();
  }

  if (bridgeBusy())
    return false;

  requestStop(true);
  return true;
}

void BridgeProcessManager::clearBridgeOutput()
{
  if (bridgeOutput_.isEmpty())
    return;

  bridgeOutput_.clear();
  emit bridgeOutputChanged();
}

QStringList BridgeProcessManager::buildFallbackArguments(const QStringList &specs)
{
  QStringList args{
      QStringLiteral("run"),
      QStringLiteral("ros_gz_bridge"),
      QStringLiteral("parameter_bridge"),
  };
  args.append(specs);
  return args;
}

QString BridgeProcessManager::buildCommand(const QStringList &specs)
{
  if (specs.isEmpty())
    return {};

  QStringList parts{
      QStringLiteral("ros2"),
      QStringLiteral("run"),
      QStringLiteral("ros_gz_bridge"),
      QStringLiteral("parameter_bridge"),
  };
  parts.append(specs);
  return parts.join(QLatin1Char(' '));
}

QString BridgeProcessManager::directExecutablePathForPrefix(const QString &prefix)
{
  if (prefix.isEmpty())
    return {};

  const QString trimmed = prefix.trimmed();
  if (trimmed.isEmpty())
    return {};

  return QDir::cleanPath(
      trimmed + QStringLiteral("/lib/ros_gz_bridge/parameter_bridge"));
}

QString BridgeProcessManager::buildDirectCommand(
    const QString &parameterBridgePath,
    const QStringList &specs)
{
  if (parameterBridgePath.isEmpty())
    return {};

  QStringList parts{parameterBridgePath};
  parts.append(specs);
  return parts.join(QLatin1Char(' '));
}

BridgeProcessManager::LaunchCommand BridgeProcessManager::buildLaunchCommand(
    const QString &parameterBridgePath,
    const QStringList &specs)
{
  LaunchCommand launch;
  if (!parameterBridgePath.isEmpty())
  {
    launch.program = parameterBridgePath;
    launch.arguments = specs;
    launch.displayCommand = buildDirectCommand(parameterBridgePath, specs);
    launch.direct = true;
    return launch;
  }

  launch.program = QStringLiteral("ros2");
  launch.arguments = buildFallbackArguments(specs);
  launch.displayCommand = buildCommand(specs);
  launch.direct = false;
  return launch;
}

QString BridgeProcessManager::statusToString(Status status)
{
  switch (status)
  {
    case Status::NotRunning:
      return QStringLiteral("Not running");
    case Status::Starting:
      return QStringLiteral("Starting");
    case Status::Running:
      return QStringLiteral("Running");
    case Status::Stopping:
      return QStringLiteral("Stopping");
    case Status::RestartRequired:
      return QStringLiteral("Restart required");
    case Status::Failed:
      return QStringLiteral("Failed");
    case Status::Crashed:
      return QStringLiteral("Crashed");
    case Status::Stopped:
      return QStringLiteral("Stopped");
    case Status::Exited:
      return QStringLiteral("Exited");
  }
  return QStringLiteral("Unknown");
}

bool BridgeProcessManager::requiresRestart(bool active,
                                           const QString &runningCommand,
                                           const QString &selectedCommand)
{
  return active &&
         !runningCommand.isEmpty() &&
         runningCommand != selectedCommand;
}

BridgeProcessManager::Status BridgeProcessManager::finalStatusForExit(
    bool userStopRequested,
    int exitCode,
    QProcess::ExitStatus exitStatus)
{
  if (userStopRequested)
    return Status::Stopped;

  if (exitStatus == QProcess::CrashExit)
    return Status::Crashed;

  if (exitCode == 0)
    return Status::Exited;

  return Status::Failed;
}

QString BridgeProcessManager::appendBoundedOutput(const QString &existing,
                                                  const QString &chunk,
                                                  int maxChars)
{
  if (maxChars <= 0)
    return {};

  const QString combined = existing + chunk;
  if (combined.size() <= maxChars)
    return combined;

  return combined.right(maxChars);
}

void BridgeProcessManager::onStarted()
{
  managedProcessId_ = static_cast<qint64>(process_->processId());
  managedProcessGroupId_ = managedProcessId_;

#ifdef Q_OS_LINUX
  if (managedProcessId_ > 0)
  {
    const pid_t pgid = ::getpgid(static_cast<pid_t>(managedProcessId_));
    if (pgid > 0)
      managedProcessGroupId_ = static_cast<qint64>(pgid);
  }
#endif

  appendOutput(QStringLiteral("Bridge process started.\n"));
  setStatus(Status::Running);
  updateRestartRequired();
}

void BridgeProcessManager::onReadyReadStandardOutput()
{
  const QString text = QString::fromUtf8(process_->readAllStandardOutput());
  if (!text.isEmpty())
    appendOutput(text);
}

void BridgeProcessManager::onReadyReadStandardError()
{
  const QString text = QString::fromUtf8(process_->readAllStandardError());
  if (!text.isEmpty())
    appendOutput(text);
}

void BridgeProcessManager::onProcessError(QProcess::ProcessError error)
{
  if (error == QProcess::UnknownError)
    return;

  if (error == QProcess::FailedToStart)
  {
    failedToStart_ = true;
    failStart(QStringLiteral("Failed to start bridge process."));
    return;
  }

  if (!process_->errorString().isEmpty())
    appendOutput(process_->errorString() + QLatin1Char('\n'));
}

void BridgeProcessManager::onFinished(int exitCode,
                                      QProcess::ExitStatus exitStatus)
{
  stopTimer_.stop();
  onReadyReadStandardOutput();
  onReadyReadStandardError();

  const bool shouldRestart = restartPending_;
  restartPending_ = false;

  if (failedToStart_)
  {
    failedToStart_ = false;
    return;
  }

  const bool stoppedByUser = userStopRequested_;
  const Status finalStatus =
      finalStatusForExit(stoppedByUser, exitCode, exitStatus);

  if (finalStatus == Status::Stopped)
    appendOutput(QStringLiteral("Bridge process stopped.\n"));
  else if (finalStatus == Status::Crashed)
    appendOutput(QStringLiteral("Bridge process crashed.\n"));
  else if (finalStatus == Status::Exited)
    appendOutput(QStringLiteral("Bridge process exited.\n"));
  else
    appendOutput(QStringLiteral("Bridge process exited with code %1.\n")
                 .arg(exitCode));

  clearRunningProcessState();
  setStatus(finalStatus);
  userStopRequested_ = false;

  if (shouldRestart)
    startDesiredBridge();
}

void BridgeProcessManager::onResolverFinished(int exitCode,
                                              QProcess::ExitStatus exitStatus)
{
  if (!resolvingExecutable_)
    return;

  resolvingExecutable_ = false;

  const QString prefix =
      QString::fromUtf8(resolverProcess_.readAllStandardOutput()).trimmed();
  const QString stderrText =
      QString::fromUtf8(resolverProcess_.readAllStandardError()).trimmed();

  if (userStopRequested_ && !restartPending_)
  {
    if (status_ == Status::Starting)
      setStatus(Status::Stopped);
    userStopRequested_ = false;
    return;
  }

  QString directExecutable;
  if (exitStatus == QProcess::NormalExit && exitCode == 0)
    directExecutable = directExecutablePathForPrefix(prefix);

  if (!directExecutable.isEmpty() &&
      QFileInfo(directExecutable).isExecutable())
  {
    resolvedParameterBridgePath_ = directExecutable;
    startBridgeWithLaunchCommand(
        buildLaunchCommand(resolvedParameterBridgePath_, desiredSpecs_));
    return;
  }

  if (!stderrText.isEmpty())
    appendOutput(stderrText + QLatin1Char('\n'));

  const QString ros2Executable = QStandardPaths::findExecutable(
      QStringLiteral("ros2"));
  if (ros2Executable.isEmpty())
  {
    failStart(QStringLiteral(
        "Failed to resolve ros_gz_bridge/parameter_bridge and ros2 is not available for fallback."));
    return;
  }

  appendOutput(QStringLiteral(
      "Warning: direct parameter_bridge resolution failed; falling back to 'ros2 run ros_gz_bridge parameter_bridge'.\n"));
  startBridgeWithLaunchCommand(buildLaunchCommand(QString{}, desiredSpecs_));
}

void BridgeProcessManager::onResolverError(QProcess::ProcessError error)
{
  if (!resolvingExecutable_)
    return;

  if (error == QProcess::UnknownError)
    return;

  resolvingExecutable_ = false;

  if (error == QProcess::FailedToStart)
  {
    failStart(QStringLiteral(
        "Failed to resolve ros_gz_bridge prefix: ros2 executable is not available."));
    return;
  }

  const QString ros2Executable = QStandardPaths::findExecutable(
      QStringLiteral("ros2"));
  if (ros2Executable.isEmpty())
  {
    failStart(QStringLiteral(
        "Failed to resolve ros_gz_bridge prefix and ros2 is not available for fallback."));
    return;
  }

  appendOutput(QStringLiteral(
      "Warning: direct parameter_bridge resolution failed; falling back to 'ros2 run ros_gz_bridge parameter_bridge'.\n"));
  startBridgeWithLaunchCommand(buildLaunchCommand(QString{}, desiredSpecs_));
}

void BridgeProcessManager::onStopTimeout()
{
  if (process_->state() == QProcess::NotRunning)
    return;

  appendOutput(QStringLiteral(
      "Bridge process did not stop after SIGINT; sending SIGKILL to the process group.\n"));
  sendSignalToManagedProcessGroup(SIGKILL);
}

void BridgeProcessManager::requestStop(bool restartAfterStop)
{
  restartPending_ = restartAfterStop;
  userStopRequested_ = true;

  if (resolvingExecutable_)
  {
    resolverProcess_.kill();
    resolverProcess_.waitForFinished(200);
    resolvingExecutable_ = false;
    setStatus(Status::Stopped);
    userStopRequested_ = false;
    if (restartAfterStop)
      startDesiredBridge();
    return;
  }

  if (process_->state() == QProcess::NotRunning)
  {
    if (restartAfterStop)
      startDesiredBridge();
    else
      setStatus(Status::Stopped);
    return;
  }

  if (status_ == Status::Stopping)
    return;

  setStatus(Status::Stopping);
  appendOutput(QStringLiteral("Stopping...\n"));
  sendSignalToManagedProcessGroup(SIGINT);
  stopTimer_.start();
}

bool BridgeProcessManager::startDesiredBridge()
{
  if (desiredSpecs_.isEmpty())
  {
    failStart(QStringLiteral("Cannot start bridge: no topics selected."));
    return false;
  }

  userStopRequested_ = false;
  failedToStart_ = false;
  setRestartRequired(false);
  setStatus(Status::Starting);

  if (!resolvedParameterBridgePath_.isEmpty() &&
      QFileInfo(resolvedParameterBridgePath_).isExecutable())
  {
    return startBridgeWithLaunchCommand(
        buildLaunchCommand(resolvedParameterBridgePath_, desiredSpecs_));
  }

  resolveExecutableAndStart();
  return true;
}

bool BridgeProcessManager::startBridgeWithLaunchCommand(
    const LaunchCommand &launchCommand)
{
  if (launchCommand.program.isEmpty())
  {
    failStart(QStringLiteral("Bridge launch command is empty."));
    return false;
  }

  if (!launchCommand.direct &&
      QStandardPaths::findExecutable(QStringLiteral("ros2")).isEmpty())
  {
    failStart(QStringLiteral(
        "Cannot start bridge fallback command because ros2 is not available."));
    return false;
  }

  runningCommandKey_ = desiredCommandKey_;
  setRunningBridgeCommand(launchCommand.displayCommand);
  appendOutput(QStringLiteral("$ ") % runningBridgeCommand_ % QLatin1Char('\n'));
  process_->setProgram(launchCommand.program);
  process_->setArguments(launchCommand.arguments);
  process_->start();
  return true;
}

void BridgeProcessManager::resolveExecutableAndStart()
{
  if (resolvingExecutable_)
    return;

  const QString ros2Executable = QStandardPaths::findExecutable(
      QStringLiteral("ros2"));
  if (ros2Executable.isEmpty())
  {
    failStart(QStringLiteral(
        "Failed to resolve ros_gz_bridge/parameter_bridge because ros2 is not available."));
    return;
  }

  resolvingExecutable_ = true;
  resolverProcess_.setProgram(ros2Executable);
  resolverProcess_.setArguments(
      {QStringLiteral("pkg"), QStringLiteral("prefix"), QStringLiteral("ros_gz_bridge")});
  resolverProcess_.start();
}

void BridgeProcessManager::setStatus(Status status)
{
  if (status_ == status)
    return;

  const bool runningBefore = bridgeRunning();
  const bool busyBefore = bridgeBusy();

  status_ = status;

  emit bridgeStatusTextChanged();

  if (runningBefore != bridgeRunning())
    emit bridgeRunningChanged();
  if (busyBefore != bridgeBusy())
    emit bridgeBusyChanged();
}

void BridgeProcessManager::setRestartRequired(bool required)
{
  if (bridgeRestartRequired_ == required)
    return;

  bridgeRestartRequired_ = required;
  emit bridgeRestartRequiredChanged();
}

void BridgeProcessManager::setRunningBridgeCommand(const QString &command)
{
  if (runningBridgeCommand_ == command)
    return;

  runningBridgeCommand_ = command;
  emit runningBridgeCommandChanged();
}

void BridgeProcessManager::clearRunningProcessState()
{
  managedProcessId_ = 0;
  managedProcessGroupId_ = 0;
  runningCommandKey_.clear();
  setRunningBridgeCommand(QString{});
  setRestartRequired(false);
}

void BridgeProcessManager::updateRestartRequired()
{
  const bool required = requiresRestart(
      bridgeRunning(),
      runningCommandKey_,
      desiredCommandKey_);

  setRestartRequired(required);

  if (status_ == Status::Running || status_ == Status::RestartRequired)
    setStatus(required ? Status::RestartRequired : Status::Running);
}

void BridgeProcessManager::appendOutput(const QString &text)
{
  if (text.isEmpty())
    return;

  const QString updated =
      appendBoundedOutput(bridgeOutput_, text, kDefaultMaxOutputChars);
  if (updated == bridgeOutput_)
    return;

  bridgeOutput_ = updated;
  emit bridgeOutputChanged();
}

void BridgeProcessManager::failStart(const QString &message)
{
  appendOutput(message + QLatin1Char('\n'));
  clearRunningProcessState();
  setStatus(Status::Failed);
  userStopRequested_ = false;
}

bool BridgeProcessManager::sendSignalToManagedProcessGroup(int signalNumber)
{
#ifdef Q_OS_LINUX
  if (managedProcessGroupId_ > 0)
  {
    if (::kill(-static_cast<pid_t>(managedProcessGroupId_), signalNumber) == 0)
      return true;
    if (errno == ESRCH)
      return false;
  }
#endif

  if (process_->state() == QProcess::NotRunning)
    return false;

  if (signalNumber == SIGKILL)
    process_->kill();
  else
    process_->terminate();
  return true;
}

void BridgeProcessManager::stopManagedProcessGroupBlocking()
{
  if (process_->state() == QProcess::NotRunning)
    return;

  process_->blockSignals(true);
  userStopRequested_ = true;
  sendSignalToManagedProcessGroup(SIGINT);
  if (!process_->waitForFinished(300))
  {
    sendSignalToManagedProcessGroup(SIGKILL);
    process_->waitForFinished(300);
  }
}

}  // namespace gz_ros2_bridge_manager
