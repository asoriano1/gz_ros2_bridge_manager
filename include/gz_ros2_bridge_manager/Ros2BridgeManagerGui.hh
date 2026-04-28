#pragma once

#include <string>
#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>

#include <gz/gui/Plugin.hh>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"
#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"
#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"
#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"

namespace gz_ros2_bridge_manager
{

class Ros2BridgeManagerGui : public gz::gui::Plugin
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
  Q_PROPERTY(QString     sensorNote            READ sensorNote            CONSTANT)

public:
  Ros2BridgeManagerGui();
  ~Ros2BridgeManagerGui() override = default;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

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
  // Returns the store key for the current selection.
  std::string currentKey() const;

  // Re-runs heuristic for current model, applies overrides, rebuilds views,
  // recomputes session command, emits topicsChanged + bridgeCommandChanged.
  void recomputeAndPublish();

  // Recomputes ONLY the session command (cheaper; used after toggling a
  // checkbox or includeAllModels). Does NOT re-run the heuristic.
  void rebuildSession();

  void setStatus(const QString &text);

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
