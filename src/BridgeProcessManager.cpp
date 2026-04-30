#include "gz_ros2_bridge_manager/BridgeProcessManager.hh"

#include <QStringBuilder>

namespace gz_ros2_bridge_manager
{

BridgeProcessManager::BridgeProcessManager()
{
  process_.setProcessChannelMode(QProcess::SeparateChannels);

  stopTimer_.setSingleShot(true);
  stopTimer_.setInterval(kStopTimeoutMs);

  connect(&process_, &QProcess::started,
          this, &BridgeProcessManager::onStarted);
  connect(&process_, &QProcess::readyReadStandardOutput,
          this, &BridgeProcessManager::onReadyReadStandardOutput);
  connect(&process_, &QProcess::readyReadStandardError,
          this, &BridgeProcessManager::onReadyReadStandardError);
  connect(&process_, &QProcess::errorOccurred,
          this, &BridgeProcessManager::onProcessError);
  connect(&process_,
          qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
          this,
          &BridgeProcessManager::onFinished);
  connect(&stopTimer_, &QTimer::timeout,
          this, &BridgeProcessManager::onStopTimeout);
}

BridgeProcessManager::~BridgeProcessManager()
{
  restartPending_ = false;
  stopTimer_.stop();

  if (process_.state() == QProcess::NotRunning)
    return;

  process_.blockSignals(true);
  process_.terminate();
  if (!process_.waitForFinished(300))
  {
    process_.kill();
    process_.waitForFinished(300);
  }
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
  desiredCommand_ = buildCommand(desiredSpecs_);
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

QStringList BridgeProcessManager::buildArguments(const QStringList &specs)
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
  appendOutput(QStringLiteral("Bridge process started.\n"));
  setStatus(Status::Running);
  updateRestartRequired();
}

void BridgeProcessManager::onReadyReadStandardOutput()
{
  const QString text = QString::fromUtf8(process_.readAllStandardOutput());
  if (!text.isEmpty())
    appendOutput(text);
}

void BridgeProcessManager::onReadyReadStandardError()
{
  const QString text = QString::fromUtf8(process_.readAllStandardError());
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
    appendOutput(QStringLiteral("Failed to start bridge process.\n"));
    if (!process_.errorString().isEmpty())
      appendOutput(process_.errorString() + QLatin1Char('\n'));
    clearRunningProcessState();
    setStatus(Status::Failed);
    return;
  }

  if (!process_.errorString().isEmpty())
    appendOutput(process_.errorString() + QLatin1Char('\n'));
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

  if (exitStatus == QProcess::CrashExit)
  {
    appendOutput(QStringLiteral("Bridge process crashed.\n"));
    setStatus(Status::Crashed);
  }
  else if (status_ == Status::Stopping)
  {
    appendOutput(QStringLiteral("Bridge process stopped.\n"));
    setStatus(Status::Exited);
  }
  else if (exitCode == 0)
  {
    appendOutput(QStringLiteral("Bridge process exited.\n"));
    setStatus(Status::Exited);
  }
  else
  {
    appendOutput(QStringLiteral("Bridge process exited with code %1.\n")
                 .arg(exitCode));
    setStatus(Status::Failed);
  }

  clearRunningProcessState();

  if (shouldRestart)
    startDesiredBridge();
}

void BridgeProcessManager::onStopTimeout()
{
  if (process_.state() == QProcess::NotRunning)
    return;

  appendOutput(
      QStringLiteral("Bridge process did not stop after terminate(); killing.\n"));
  process_.kill();
}

void BridgeProcessManager::requestStop(bool restartAfterStop)
{
  restartPending_ = restartAfterStop;

  if (process_.state() == QProcess::NotRunning)
  {
    if (restartAfterStop)
      startDesiredBridge();
    return;
  }

  if (status_ == Status::Stopping)
    return;

  setStatus(Status::Stopping);
  process_.terminate();
  stopTimer_.start();
}

bool BridgeProcessManager::startDesiredBridge()
{
  if (desiredSpecs_.isEmpty())
  {
    appendOutput(QStringLiteral("Cannot start bridge: no topics selected.\n"));
    clearRunningProcessState();
    setStatus(Status::Failed);
    return false;
  }

  setRunningBridgeCommand(desiredCommand_);
  setRestartRequired(false);
  failedToStart_ = false;

  appendOutput(QStringLiteral("$ ") % runningBridgeCommand_ % QLatin1Char('\n'));
  process_.setProgram(QStringLiteral("ros2"));
  process_.setArguments(buildArguments(desiredSpecs_));
  setStatus(Status::Starting);
  process_.start();
  return true;
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
  setRunningBridgeCommand(QString{});
  setRestartRequired(false);
}

void BridgeProcessManager::updateRestartRequired()
{
  const bool required = requiresRestart(
      process_.state() != QProcess::NotRunning || bridgeRunning(),
      runningBridgeCommand_,
      desiredCommand_);

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

}  // namespace gz_ros2_bridge_manager
