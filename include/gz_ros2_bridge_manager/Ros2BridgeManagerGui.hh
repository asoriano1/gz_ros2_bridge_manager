#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>

#include <gz/sim/gui/GuiSystem.hh>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"
#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"
#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"
#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"

namespace gz_ros2_bridge_manager
{

/// Gazebo GUI plugin.
///
/// Inherits from gz::sim::GuiSystem (which extends gz::gui::Plugin) so that
/// the ECM Update() callback is invoked on every simulation step.  All Qt
/// interactions are guarded to the main thread via Qt::QueuedConnection.
class Ros2BridgeManagerGui : public gz::sim::GuiSystem
{
  Q_OBJECT

  Q_PROPERTY(QString     worldName             READ worldName             NOTIFY worldNameChanged)
  Q_PROPERTY(QStringList modelNames            READ modelNames            NOTIFY modelNamesChanged)
  Q_PROPERTY(QString     selectedModel         READ selectedModel         NOTIFY selectedModelChanged)

  Q_PROPERTY(QVariantList associatedTopics     READ associatedTopics      NOTIFY topicsChanged)
  Q_PROPERTY(QVariantList unassignedTopics     READ unassignedTopics      NOTIFY topicsChanged)
  Q_PROPERTY(QVariantList unsupportedTopics    READ unsupportedTopics     NOTIFY topicsChanged)

  Q_PROPERTY(QString     bridgeCommand         READ bridgeCommand         NOTIFY bridgeCommandChanged)
  Q_PROPERTY(QString     bridgeCommandDisplay  READ bridgeCommandDisplay  NOTIFY bridgeCommandChanged)
  Q_PROPERTY(QString     selectionSummary      READ selectionSummary      NOTIFY bridgeCommandChanged)
  Q_PROPERTY(QString     missingTopicsWarning  READ missingTopicsWarning  NOTIFY bridgeCommandChanged)
  Q_PROPERTY(int         checkedCurrentModelCount READ checkedCurrentModelCount NOTIFY bridgeCommandChanged)
  Q_PROPERTY(int         checkedAllModelsCount    READ checkedAllModelsCount    NOTIFY bridgeCommandChanged)

  Q_PROPERTY(bool        includeAllModels      READ includeAllModels      NOTIFY includeAllModelsChanged)
  Q_PROPERTY(bool        autoRefresh           READ autoRefresh           NOTIFY autoRefreshChanged)
  Q_PROPERTY(QString     lastRefreshTime       READ lastRefreshTime       NOTIFY lastRefreshTimeChanged)

  Q_PROPERTY(QString     statusText            READ statusText            NOTIFY statusTextChanged)
  Q_PROPERTY(QString     modelGoneWarning      READ modelGoneWarning      NOTIFY modelGoneWarningChanged)
  Q_PROPERTY(bool        busy                  READ busy                  NOTIFY busyChanged)
  Q_PROPERTY(bool        hasBridgeableTopics   READ hasBridgeableTopics   NOTIFY topicsChanged)
  Q_PROPERTY(QStringList warnings              READ warnings              NOTIFY topicsChanged)
  Q_PROPERTY(QString     sensorNote            READ sensorNote            NOTIFY topicsChanged)

  // ECM-confirmed sensor hierarchy.
  Q_PROPERTY(bool        ecmAvailable          READ ecmAvailable          NOTIFY topicsChanged)
  Q_PROPERTY(QVariantList sensorTree           READ sensorTree            NOTIFY topicsChanged)

  // ECM status counters (updated each time ECM fingerprint changes).
  Q_PROPERTY(int         ecmModelCount                 READ ecmModelCount                 NOTIFY topicsChanged)
  Q_PROPERTY(int         ecmSensorCount                READ ecmSensorCount                NOTIFY topicsChanged)
  Q_PROPERTY(int         selectedModelSensorCount      READ selectedModelSensorCount      NOTIFY topicsChanged)
  Q_PROPERTY(int         selectedModelMatchedTopicCount READ selectedModelMatchedTopicCount NOTIFY topicsChanged)
  Q_PROPERTY(int         selectedModelUnresolvedSensorCount READ selectedModelUnresolvedSensorCount NOTIFY topicsChanged)
  Q_PROPERTY(QString     sensorDiscoveryStatus         READ sensorDiscoveryStatus         NOTIFY topicsChanged)

public:
  Ros2BridgeManagerGui();
  ~Ros2BridgeManagerGui() override = default;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  // GuiSystem override — called from the Gazebo update thread every sim step.
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  // ---- Property reads ----
  QString      worldName()             const { return worldName_; }
  QStringList  modelNames()            const { return modelNames_; }
  QString      selectedModel()         const { return selectedModel_; }
  QVariantList associatedTopics()      const { return associatedView_; }
  QVariantList unassignedTopics()      const { return unassignedView_; }
  QVariantList unsupportedTopics()     const { return unsupportedView_; }
  QString      bridgeCommand()         const { return bridgeCommand_; }
  QString      bridgeCommandDisplay()  const { return bridgeCommandDisplay_; }
  QString      selectionSummary()      const { return selectionSummary_; }
  QString      missingTopicsWarning()  const { return missingTopicsWarning_; }
  int          checkedCurrentModelCount() const { return currentChecked_; }
  int          checkedAllModelsCount() const    { return totalChecked_; }
  bool         includeAllModels()      const { return includeAllModels_; }
  bool         autoRefresh()           const { return autoRefresh_; }
  QString      lastRefreshTime()       const { return lastRefreshTime_; }
  QString      statusText()            const { return statusText_; }
  QString      modelGoneWarning()      const { return modelGoneWarning_; }
  bool         busy()                  const { return busy_; }
  bool         hasBridgeableTopics()   const;
  QStringList  warnings()              const { return warnings_; }
  QString      sensorNote()            const;
  bool         ecmAvailable()          const { return ecmAvailable_; }
  QVariantList sensorTree()            const { return sensorTree_; }

  int     ecmModelCount()                      const { return ecmModelCount_; }
  int     ecmSensorCount()                     const { return ecmSensorCount_; }
  int     selectedModelSensorCount()           const { return selectedModelSensorCount_; }
  int     selectedModelMatchedTopicCount()     const { return selectedModelMatchedTopicCount_; }
  int     selectedModelUnresolvedSensorCount() const { return selectedModelUnresolvedSensorCount_; }
  QString sensorDiscoveryStatus()              const { return sensorDiscoveryStatus_; }

  // ---- Invokables ----
  Q_INVOKABLE void refresh();
  Q_INVOKABLE void selectModel(const QString &modelName);
  Q_INVOKABLE void setTopicChecked(const QString &topic, bool checked);
  Q_INVOKABLE void copyBridgeCommand();
  Q_INVOKABLE void checkAllAssociated();
  Q_INVOKABLE void uncheckAllCurrentModel();
  Q_INVOKABLE void resetCurrentModelSelection();
  Q_INVOKABLE void setIncludeAllModels(bool enabled);
  Q_INVOKABLE void setAutoRefresh(bool enabled);

signals:
  void worldNameChanged();
  void modelNamesChanged();
  void selectedModelChanged();
  void topicsChanged();
  void bridgeCommandChanged();
  void statusTextChanged();
  void modelGoneWarningChanged();
  void busyChanged();
  void includeAllModelsChanged();
  void autoRefreshChanged();
  void lastRefreshTimeChanged();

private slots:
  void onAutoRefreshTick();

private:
  std::string currentKey() const;
  void recomputeAndPublish();
  void rebuildSession();
  void setStatus(const QString &text);

  // ---- ECM snapshot (written by Update() thread, read by main thread) ----
  // All accesses to ecmSensors_ / ecmFingerprint_ must hold ecmMutex_.
  std::mutex ecmMutex_;
  std::vector<EcmSensorEntry> ecmSensors_;
  size_t ecmFingerprint_{0};  // fingerprint-based change detection
  std::atomic<bool> ecmUpdatePending_{false};

  // ---- Discovery snapshot (set on main thread after worker finishes) ----
  std::vector<GzTopicEntry> discoveredTopics_;
  std::vector<std::string>  discoveredModels_;

  // ---- Selection state ----
  QString  worldName_;
  QString  selectedModel_;
  QStringList modelNames_;

  ModelTopicSelectionStore store_;

  // ---- Heuristic-derived candidates for current selection ----
  std::vector<BridgeTopicCandidate> associatedCands_;
  std::vector<BridgeTopicCandidate> unassignedCands_;
  std::vector<BridgeTopicCandidate> unsupportedCands_;

  // ---- QML-facing views ----
  QVariantList associatedView_;
  QVariantList unassignedView_;
  QVariantList unsupportedView_;

  // ---- ECM sensor tree (main thread, rebuilt in recomputeAndPublish) ----
  bool         ecmAvailable_{false};
  QVariantList sensorTree_;

  // ---- ECM status counters (main thread, updated in recomputeAndPublish) ----
  int     ecmModelCount_{0};
  int     ecmSensorCount_{0};
  int     selectedModelSensorCount_{0};
  int     selectedModelMatchedTopicCount_{0};
  int     selectedModelUnresolvedSensorCount_{0};
  QString sensorDiscoveryStatus_;

  // ---- Generated commands & summaries ----
  QString  bridgeCommand_;
  QString  bridgeCommandDisplay_;
  QString  selectionSummary_;
  QString  missingTopicsWarning_;
  int      currentChecked_ = 0;
  int      totalChecked_   = 0;

  // ---- Settings & status ----
  bool     includeAllModels_ = false;
  bool     autoRefresh_      = false;
  QString  lastRefreshTime_;
  QString  statusText_{"Not yet refreshed"};
  QString  modelGoneWarning_;
  QStringList warnings_;
  bool     busy_             = false;

  QTimer   autoRefreshTimer_;

  BridgeTypeMapper           mapper_;
  TopicAssociationHeuristic  heuristic_;
};

}  // namespace gz_ros2_bridge_manager
