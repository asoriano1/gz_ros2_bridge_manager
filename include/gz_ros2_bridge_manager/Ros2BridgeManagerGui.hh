#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>

#include <gz/sim/gui/GuiSystem.hh>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"
#include "gz_ros2_bridge_manager/BridgeProcessManager.hh"
#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"
#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"

namespace gz_ros2_bridge_manager
{

/// Gazebo GUI plugin — ECM-first multi-model accordion workflow.
///
/// Inherits from gz::sim::GuiSystem so that the ECM Update() callback is
/// invoked on every simulation step.  All Qt interactions are guarded to the
/// main thread via Qt::QueuedConnection.
class Ros2BridgeManagerGui : public gz::sim::GuiSystem
{
  Q_OBJECT

  // ---- Status / settings ----
  Q_PROPERTY(QString      worldName             READ worldName             NOTIFY worldNameChanged)
  Q_PROPERTY(QString      statusText            READ statusText            NOTIFY statusTextChanged)
  Q_PROPERTY(bool         busy                  READ busy                  NOTIFY busyChanged)
  Q_PROPERTY(bool         autoRefresh           READ autoRefresh           NOTIFY autoRefreshChanged)
  Q_PROPERTY(QString      lastRefreshTime       READ lastRefreshTime       NOTIFY lastRefreshTimeChanged)

  // ---- Multi-model accordion ----
  // Each element is a QVariantMap:
  //   modelName, ecmAvailable, ecmSensorCount, unresolvedSensorCount,
  //   selectedTopicCount,
  //   sensors (QVariantList of sensorToVariantMap + per-topic checked/bridgeable)
  Q_PROPERTY(QVariantList modelCards                READ modelCards                NOTIFY modelsChanged)
  Q_PROPERTY(QVariantList additionalBridgeableTopics READ additionalBridgeableTopics NOTIFY modelsChanged)
  Q_PROPERTY(QVariantList unsupportedTopics         READ unsupportedTopics         NOTIFY modelsChanged)
  Q_PROPERTY(QStringList  warnings                  READ warnings                  NOTIFY modelsChanged)

  // ---- Bridge command ----
  Q_PROPERTY(QString      bridgeCommand             READ bridgeCommand             NOTIFY bridgeCommandChanged)
  Q_PROPERTY(QString      bridgeCommandDisplay      READ bridgeCommandDisplay      NOTIFY bridgeCommandChanged)
  Q_PROPERTY(int          selectedBridgeTopicCount  READ selectedBridgeTopicCount  NOTIFY bridgeCommandChanged)
  Q_PROPERTY(bool         bridgeRunning             READ bridgeRunning             NOTIFY bridgeRunningChanged)
  Q_PROPERTY(bool         bridgeBusy                READ bridgeBusy                NOTIFY bridgeBusyChanged)
  Q_PROPERTY(QString      bridgeStatusText          READ bridgeStatusText          NOTIFY bridgeStatusTextChanged)
  Q_PROPERTY(QString      bridgeOutput              READ bridgeOutput              NOTIFY bridgeOutputChanged)
  Q_PROPERTY(bool         bridgeRestartRequired     READ bridgeRestartRequired     NOTIFY bridgeRestartRequiredChanged)
  Q_PROPERTY(QString      runningBridgeCommand      READ runningBridgeCommand      NOTIFY runningBridgeCommandChanged)

public:
  Ros2BridgeManagerGui();
  ~Ros2BridgeManagerGui() override = default;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  // GuiSystem override — called from the Gazebo update thread every sim step.
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  // ---- Property reads ----
  QString      worldName()          const { return worldName_; }
  QString      statusText()         const { return statusText_; }
  bool         busy()               const { return busy_; }
  bool         autoRefresh()        const { return autoRefresh_; }
  QString      lastRefreshTime()    const { return lastRefreshTime_; }
  QVariantList modelCards()         const { return modelCards_; }
  QVariantList additionalBridgeableTopics() const { return additionalView_; }
  QVariantList unsupportedTopics()  const { return unsupportedView_; }
  QStringList  warnings()           const { return warnings_; }
  QString      bridgeCommand()      const { return bridgeCommand_; }
  QString      bridgeCommandDisplay() const { return bridgeCommandDisplay_; }
  int          selectedBridgeTopicCount() const { return selectedBridgeTopicCount_; }
  bool         bridgeRunning()      const { return bridgeProcess_.bridgeRunning(); }
  bool         bridgeBusy()         const { return bridgeProcess_.bridgeBusy(); }
  QString      bridgeStatusText()   const { return bridgeProcess_.bridgeStatusText(); }
  QString      bridgeOutput()       const { return bridgeProcess_.bridgeOutput(); }
  bool         bridgeRestartRequired() const { return bridgeProcess_.bridgeRestartRequired(); }
  QString      runningBridgeCommand() const { return bridgeProcess_.runningBridgeCommand(); }

  // ---- Invokables ----
  Q_INVOKABLE void refresh();
  Q_INVOKABLE void setTopicChecked(const QString &modelName, const QString &topic, bool checked);
  Q_INVOKABLE void setAdditionalTopicChecked(const QString &topic, bool checked);
  Q_INVOKABLE void resetModelSelection(const QString &modelName);
  Q_INVOKABLE void copyBridgeCommand();
  Q_INVOKABLE void setAutoRefresh(bool enabled);
  Q_INVOKABLE void runBridge();
  Q_INVOKABLE void stopBridge();
  Q_INVOKABLE void restartBridge();
  Q_INVOKABLE void clearBridgeOutput();

signals:
  void worldNameChanged();
  void statusTextChanged();
  void busyChanged();
  void autoRefreshChanged();
  void lastRefreshTimeChanged();
  void modelsChanged();
  void bridgeCommandChanged();
  void bridgeRunningChanged();
  void bridgeBusyChanged();
  void bridgeStatusTextChanged();
  void bridgeOutputChanged();
  void bridgeRestartRequiredChanged();
  void runningBridgeCommandChanged();

private slots:
  void onAutoRefreshTick();

private:
  void recomputeAndPublish();
  void rebuildModelCards();
  void rebuildBridgeCommand();
  void setStatus(const QString &text);
  QVariantMap buildModelCard(const std::string &modelName,
                              const ModelSensorTree &tree,
                              const std::vector<BridgeTopicCandidate> &cands);

  // ---- ECM snapshot (written by Update() thread, read by main thread) ----
  // All accesses to ecmSensors_ / ecmFingerprint_ must hold ecmMutex_.
  std::mutex ecmMutex_;
  std::vector<EcmSensorEntry> ecmSensors_;
  size_t ecmFingerprint_{0};
  std::atomic<bool> ecmUpdatePending_{false};

  // ---- Discovery snapshot (set on main thread after worker finishes) ----
  std::vector<GzTopicEntry> discoveredTopics_;
  std::vector<std::string>  discoveredModels_;

  // ---- World state ----
  QString worldName_;

  ModelTopicSelectionStore store_;

  // ---- Per-model data (main thread, rebuilt in recomputeAndPublish) ----
  std::unordered_map<std::string, std::vector<BridgeTopicCandidate>> perModelCands_;
  std::unordered_map<std::string, ModelSensorTree>                   perModelTrees_;
  std::vector<BridgeTopicCandidate> additionalCands_;
  std::vector<BridgeTopicCandidate> unsupportedCands_;

  // ---- QML-facing views ----
  QVariantList modelCards_;
  QVariantList additionalView_;
  QVariantList unsupportedView_;
  QStringList  warnings_;

  // ---- Generated command ----
  QString bridgeCommand_;
  QString bridgeCommandDisplay_;
  int     selectedBridgeTopicCount_{0};

  // ---- Settings & status ----
  bool    autoRefresh_{false};
  QString lastRefreshTime_;
  QString statusText_{"Not yet refreshed"};
  bool    busy_{false};

  QTimer       autoRefreshTimer_;
  BridgeTypeMapper mapper_;
  BridgeProcessManager bridgeProcess_;
};

}  // namespace gz_ros2_bridge_manager
