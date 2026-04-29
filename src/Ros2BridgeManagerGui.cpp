#include "gz_ros2_bridge_manager/Ros2BridgeManagerGui.hh"

#include <QtConcurrent/QtConcurrentRun>
#include <QClipboard>
#include <QDateTime>
#include <QGuiApplication>
#include <QVariantMap>

#include <gz/gui/Application.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include <gz/sim/EntityComponentManager.hh>

#include "gz_ros2_bridge_manager/BridgeSession.hh"
#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"
#include "gz_ros2_bridge_manager/EcmSensorExtractor.hh"
#include "gz_ros2_bridge_manager/ModelSensorDiscovery.hh"
#include "gz_ros2_bridge_manager/WorldDiscovery.hh"

// Q_INIT_RESOURCE must live outside any namespace.
static void initBridgeManagerResources()
{
  Q_INIT_RESOURCE(Ros2BridgeManagerGui);
}

namespace gz_ros2_bridge_manager
{

namespace
{

constexpr int kAutoRefreshIntervalMs = 2500;

// ---- QML helpers --------------------------------------------------------

QVariantMap toVariantMap(const BridgeTopicCandidate &c)
{
  QVariantMap m;
  m[QStringLiteral("topic")]      = QString::fromStdString(c.gzTopic);
  m[QStringLiteral("gzType")]     = QString::fromStdString(c.gzType);
  m[QStringLiteral("ros2Type")]   = QString::fromStdString(c.ros2Type);
  m[QStringLiteral("bridgeSpec")] = QString::fromStdString(c.bridgeSpec);
  m[QStringLiteral("category")]   = QString::fromLatin1(categoryName(c.category));
  m[QStringLiteral("confidence")] = QString::fromStdString(c.confidenceLabel);
  m[QStringLiteral("bridgeable")] = c.bridgeable;
  m[QStringLiteral("checked")]    = c.checked;
  m[QStringLiteral("ambiguous")]  = c.ambiguous;
  m[QStringLiteral("isGeneric")]  = c.isGeneric;
  m[QStringLiteral("warning")]    = QString::fromStdString(c.warning);
  return m;
}

QVariantList toVariantList(const std::vector<BridgeTopicCandidate> &cs)
{
  QVariantList list;
  list.reserve(static_cast<int>(cs.size()));
  for (const auto &c : cs)
    list.append(toVariantMap(c));
  return list;
}

QVariantMap sensorToVariantMap(const DiscoveredSensor &ds)
{
  QVariantMap m;
  m[QStringLiteral("modelName")]    = QString::fromStdString(ds.sensor.modelName);
  m[QStringLiteral("linkName")]     = QString::fromStdString(ds.sensor.linkName);
  m[QStringLiteral("sensorName")]   = QString::fromStdString(ds.sensor.sensorName);
  m[QStringLiteral("sensorType")]   = QString::fromStdString(ds.sensor.sensorType);
  m[QStringLiteral("declaredTopic")]= QString::fromStdString(ds.sensor.declaredTopic);
  m[QStringLiteral("matchSource")]  = QString::fromLatin1(matchSourceName(ds.matchSource));
  m[QStringLiteral("nestedModel")]  = ds.sensor.nestedModel;
  m[QStringLiteral("resolved")]     = ds.resolved;
  m[QStringLiteral("warning")]      = QString::fromStdString(ds.warning);
  m[QStringLiteral("topicCount")]   = static_cast<int>(ds.matchedTopicNames.size());
  QStringList specs;
  for (const auto &s : ds.matchedTopicNames)
    specs << QString::fromStdString(s);
  m[QStringLiteral("matchedTopics")] = specs;
  return m;
}

// Build BridgeTopicCandidate entries from ECM-matched sensor topics.
// Topics already covered by ECM are excluded from the heuristic below.
std::vector<BridgeTopicCandidate> ecmToCandidates(
    const ModelSensorTree &tree,
    const BridgeTypeMapper & /*mapper*/,
    std::unordered_set<std::string> &coveredTopics)
{
  std::vector<BridgeTopicCandidate> result;
  std::unordered_set<std::string> seenSpecs;

  for (const auto &ds : tree.sensors)
  {
    for (size_t i = 0; i < ds.matchedTopicNames.size(); ++i)
    {
      const auto &topicName = ds.matchedTopicNames[i];
      const auto &spec      = ds.matchedBridgeSpecs[i];

      if (!seenSpecs.insert(spec).second)
        continue;
      coveredTopics.insert(topicName);

      BridgeTopicCandidate c;
      c.gzTopic    = topicName;
      c.bridgeSpec = spec;
      c.bridgeable = true;
      c.checked    = true;  // ECM-confirmed → checked by default
      c.category   = AssociationCategory::EcmConfirmed;
      c.confidenceLabel = "ECM confirmed";
      c.isGeneric  = false;

      // Parse gz type and ros2 type from bridge spec (<topic>@<ros2type>@<gztype>)
      const auto at1 = spec.find('@');
      if (at1 != std::string::npos)
      {
        const auto at2 = spec.find('@', at1 + 1);
        if (at2 != std::string::npos)
        {
          c.ros2Type = spec.substr(at1 + 1, at2 - at1 - 1);
          c.gzType   = spec.substr(at2 + 1);
        }
      }

      c.warning = ds.warning;
      result.push_back(std::move(c));
    }

    // Unresolved sensor: emit one placeholder so the sensor is visible.
    if (!ds.resolved)
    {
      BridgeTopicCandidate c;
      // Use fallback prefix if available, otherwise the declared topic.
      c.gzTopic = !ds.sensor.declaredTopic.empty()
                    ? ds.sensor.declaredTopic
                    : ds.sensor.fallbackGazeboTopicPrefix;
      c.bridgeable      = false;
      c.checked         = false;
      c.category        = AssociationCategory::EcmConfirmed;
      c.confidenceLabel = "ECM (no topic yet)";
      c.warning         = ds.warning;
      result.push_back(std::move(c));
    }
  }
  return result;
}

}  // namespace

// ============================================================================
// Construction
// ============================================================================

Ros2BridgeManagerGui::Ros2BridgeManagerGui()
: gz::sim::GuiSystem()
{
  initBridgeManagerResources();

  if (auto *engine = gz::gui::App()->Engine())
  {
    engine->rootContext()->setContextProperty(
        QStringLiteral("bridgeManager"), this);
  }

  autoRefreshTimer_.setInterval(kAutoRefreshIntervalMs);
  connect(&autoRefreshTimer_, &QTimer::timeout,
          this, &Ros2BridgeManagerGui::onAutoRefreshTick);
}

void Ros2BridgeManagerGui::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "ROS 2 Bridge Manager";
}

// ============================================================================
// GuiSystem::Update — called from the Gazebo update thread
// ============================================================================

void Ros2BridgeManagerGui::Update(const gz::sim::UpdateInfo & /*_info*/,
                                   gz::sim::EntityComponentManager &_ecm)
{
  const size_t newFingerprint = EcmSensorExtractor::computeFingerprint(_ecm);

  {
    std::lock_guard<std::mutex> lk(ecmMutex_);
    if (newFingerprint == ecmFingerprint_)
      return;
  }

  // Fingerprint changed — do full extraction without holding the lock.
  std::vector<EcmSensorEntry> newSensors = EcmSensorExtractor::extract(_ecm);

  gzdbg << "[BridgeManager] ECM sensor state changed: "
        << newSensors.size() << " sensor(s) detected\n";

  bool shouldNotify = false;
  {
    std::lock_guard<std::mutex> lk(ecmMutex_);
    ecmSensors_     = std::move(newSensors);
    ecmFingerprint_ = newFingerprint;
    if (!ecmUpdatePending_.exchange(true))
      shouldNotify = true;
  }

  if (shouldNotify)
  {
    QMetaObject::invokeMethod(this,
      [this]()
      {
        ecmUpdatePending_ = false;
        recomputeAndPublish();
      }, Qt::QueuedConnection);
  }
}

// ============================================================================
// Helpers
// ============================================================================

std::string Ros2BridgeManagerGui::currentKey() const
{
  const std::string world = worldName_.toStdString();
  if (selectedModel_.isEmpty())
    return ModelTopicSelectionStore::manualKey(world);
  return ModelTopicSelectionStore::keyForModel(
      world, selectedModel_.toStdString());
}

bool Ros2BridgeManagerGui::hasBridgeableTopics() const
{
  return !associatedCands_.empty() || !unassignedCands_.empty();
}

QString Ros2BridgeManagerGui::sensorNote() const
{
  if (ecmAvailable_)
    return QStringLiteral("Sensor hierarchy confirmed via ECM.");
  return QString::fromStdString(ModelSensorDiscovery::unavailableReason());
}

void Ros2BridgeManagerGui::setStatus(const QString &text)
{
  if (statusText_ == text) return;
  statusText_ = text;
  emit statusTextChanged();
}

// ============================================================================
// Recompute pipelines
// ============================================================================

void Ros2BridgeManagerGui::recomputeAndPublish()
{
  // ---- Grab ECM snapshot (thread-safe copy) --------------------------------
  std::vector<EcmSensorEntry> ecmSensors;
  {
    std::lock_guard<std::mutex> lk(ecmMutex_);
    ecmSensors = ecmSensors_;
  }

  // ---- ECM status counters ------------------------------------------------
  {
    std::unordered_set<EntityId> modelIds;
    for (const auto &s : ecmSensors)
      if (s.modelEntity != 0) modelIds.insert(s.modelEntity);
    ecmModelCount_  = static_cast<int>(modelIds.size());
    ecmSensorCount_ = static_cast<int>(ecmSensors.size());
  }

  // ---- ECM-confirmed sensor matching for selected model -------------------
  const std::string world = worldName_.toStdString();
  const std::string model = selectedModel_.toStdString();

  ModelSensorTree sensorTree = EcmTopicMatcher::matchAll(
      world, ecmSensors, model, discoveredTopics_);

  // Count per-selection stats.
  int matchedTopics  = 0;
  int unresolvedSensors = 0;
  for (const auto &ds : sensorTree.sensors)
  {
    matchedTopics += static_cast<int>(ds.matchedTopicNames.size());
    if (!ds.resolved) ++unresolvedSensors;
  }
  selectedModelSensorCount_           = static_cast<int>(sensorTree.sensors.size());
  selectedModelMatchedTopicCount_     = matchedTopics;
  selectedModelUnresolvedSensorCount_ = unresolvedSensors;

  // Build sensorDiscoveryStatus string.
  if (ecmSensors.empty())
  {
    sensorDiscoveryStatus_ = QStringLiteral("No sensors detected in ECM");
  }
  else
  {
    sensorDiscoveryStatus_ =
        QString("%1 sensor%2 across %3 model%4")
            .arg(ecmSensorCount_)
            .arg(ecmSensorCount_ == 1 ? "" : "s")
            .arg(ecmModelCount_)
            .arg(ecmModelCount_ == 1 ? "" : "s");
    if (!model.empty())
    {
      sensorDiscoveryStatus_ +=
          QString(" — %1 in selection (%2 resolved, %3 unresolved)")
              .arg(selectedModelSensorCount_)
              .arg(selectedModelSensorCount_ - unresolvedSensors)
              .arg(unresolvedSensors);
    }
  }

  // Topics already claimed by ECM — excluded from heuristic below.
  std::unordered_set<std::string> coveredTopics;
  std::vector<BridgeTopicCandidate> ecmCands =
      ecmToCandidates(sensorTree, mapper_, coveredTopics);

  // Apply store overrides to ECM candidates (respect user's unchecks).
  const std::string key = currentKey();
  store_.applyOverrides(key, ecmCands);

  ecmAvailable_ = !sensorTree.sensors.empty();

  // Build sensorTree QML view.
  sensorTree_.clear();
  for (const auto &ds : sensorTree.sensors)
    sensorTree_.append(sensorToVariantMap(ds));

  // ---- Heuristic for topics not covered by ECM ----------------------------
  auto result = heuristic_.associate(
      model, world, discoveredTopics_, discoveredModels_);

  // Remove topics already in ECM from heuristic results.
  auto filterCovered = [&](std::vector<BridgeTopicCandidate> &cs)
  {
    cs.erase(
      std::remove_if(cs.begin(), cs.end(),
        [&](const BridgeTopicCandidate &c) {
          return coveredTopics.count(c.gzTopic) > 0;
        }),
      cs.end());
  };
  filterCovered(result.associated);
  filterCovered(result.unassigned);

  store_.applyOverrides(key, result.associated);
  store_.applyOverrides(key, result.unassigned);

  // ---- Merge: ECM-confirmed first, then heuristic -------------------------
  associatedCands_.clear();
  associatedCands_.insert(associatedCands_.end(),
                          ecmCands.begin(), ecmCands.end());
  associatedCands_.insert(associatedCands_.end(),
                          result.associated.begin(), result.associated.end());

  unassignedCands_  = std::move(result.unassigned);
  unsupportedCands_ = std::move(result.unsupported);

  associatedView_  = toVariantList(associatedCands_);
  unassignedView_  = toVariantList(unassignedCands_);
  unsupportedView_ = toVariantList(unsupportedCands_);

  warnings_.clear();
  for (const auto &w : result.warnings)
    warnings_ << QString::fromStdString(w);

  emit topicsChanged();

  rebuildSession();
}

void Ros2BridgeManagerGui::rebuildSession()
{
  std::vector<BridgeTopicCandidate> currentCands;
  currentCands.reserve(associatedCands_.size() + unassignedCands_.size());
  currentCands.insert(currentCands.end(),
                      associatedCands_.begin(), associatedCands_.end());
  currentCands.insert(currentCands.end(),
                      unassignedCands_.begin(), unassignedCands_.end());

  const auto session = BridgeSessionBuilder::build(
      worldName_.toStdString(),
      currentKey(),
      currentCands,
      store_,
      heuristic_,
      discoveredTopics_,
      discoveredModels_,
      includeAllModels_);

  bridgeCommand_        = QString::fromStdString(session.command);
  bridgeCommandDisplay_ = QString::fromStdString(session.commandWrapped);
  currentChecked_       = session.currentModelChecked;
  totalChecked_         = session.currentModelChecked + session.otherModelsChecked;

  if (includeAllModels_ && session.otherModelsChecked > 0)
  {
    selectionSummary_ = QString("%1 from current  +  %2 from other models")
                          .arg(session.currentModelChecked)
                          .arg(session.otherModelsChecked);
  }
  else
  {
    selectionSummary_ = QString("%1 topic%2 selected")
                          .arg(session.currentModelChecked)
                          .arg(session.currentModelChecked == 1 ? "" : "s");
  }

  if (session.missingTopics.empty())
  {
    missingTopicsWarning_.clear();
  }
  else
  {
    QStringList list;
    for (const auto &t : session.missingTopics)
      list << QString::fromStdString(t);
    missingTopicsWarning_ =
      QString("%1 previously selected topic%2 not currently advertised: %3")
        .arg(list.size())
        .arg(list.size() == 1 ? "" : "s")
        .arg(list.join(", "));
  }

  emit bridgeCommandChanged();
}

// ============================================================================
// Refresh — discover everything, then recompute + publish
// ============================================================================

void Ros2BridgeManagerGui::refresh()
{
  if (busy_) return;

  busy_ = true;
  emit busyChanged();
  setStatus(QStringLiteral("Discovering…"));

  Ros2BridgeManagerGui *self = this;

  QtConcurrent::run([self]()
  {
    WorldDiscovery       worldDisc;
    GazeboTopicDiscovery topicDisc;

    const WorldInfo worldInfo = worldDisc.discover();
    const auto      topics    = topicDisc.discover(self->mapper_);

    QMetaObject::invokeMethod(self,
      [self, worldInfo, topics]()
      {
        const QString newWorld = QString::fromStdString(worldInfo.worldName);
        if (newWorld != self->worldName_)
        {
          self->worldName_ = newWorld;
          emit self->worldNameChanged();
        }

        QStringList modelNames;
        for (const auto &m : worldInfo.modelNames)
          modelNames << QString::fromStdString(m);

        if (modelNames != self->modelNames_)
        {
          self->modelNames_ = modelNames;
          emit self->modelNamesChanged();
        }
        self->discoveredModels_ = worldInfo.modelNames;

        if (!self->selectedModel_.isEmpty() &&
            !modelNames.contains(self->selectedModel_))
        {
          self->modelGoneWarning_ = QString(
              "Selected model '%1' is no longer present in the world. "
              "Switched to manual mode; selection state preserved.")
                .arg(self->selectedModel_);
          self->selectedModel_.clear();
          emit self->selectedModelChanged();
          emit self->modelGoneWarningChanged();
        }
        else if (!self->modelGoneWarning_.isEmpty() &&
                 !self->selectedModel_.isEmpty())
        {
          self->modelGoneWarning_.clear();
          emit self->modelGoneWarningChanged();
        }

        self->discoveredTopics_ = topics;

        if (!worldInfo.errorMessage.empty())
        {
          self->setStatus(QString::fromStdString(worldInfo.errorMessage));
        }
        else
        {
          self->setStatus(
              QString("World: %1  •  Models: %2  •  Topics: %3")
                  .arg(self->worldName_)
                  .arg(modelNames.size())
                  .arg(static_cast<int>(topics.size())));
        }

        self->lastRefreshTime_ =
          QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));
        emit self->lastRefreshTimeChanged();

        self->recomputeAndPublish();

        self->busy_ = false;
        emit self->busyChanged();
      }, Qt::QueuedConnection);
  });
}

// ============================================================================
// Selection / per-topic toggles
// ============================================================================

void Ros2BridgeManagerGui::selectModel(const QString &modelName)
{
  if (selectedModel_ == modelName)
    return;
  selectedModel_ = modelName;
  if (!selectedModel_.isEmpty() && !modelGoneWarning_.isEmpty())
  {
    modelGoneWarning_.clear();
    emit modelGoneWarningChanged();
  }
  emit selectedModelChanged();
  recomputeAndPublish();
}

void Ros2BridgeManagerGui::setTopicChecked(const QString &topic, bool checked)
{
  const std::string key    = currentKey();
  const std::string sTopic = topic.toStdString();

  bool bridgeable = false;
  auto applyTo = [&](std::vector<BridgeTopicCandidate> &cs)
  {
    for (auto &c : cs)
    {
      if (c.gzTopic == sTopic)
      {
        if (!c.bridgeable) return false;
        c.checked = checked;
        bridgeable = true;
        return true;
      }
    }
    return false;
  };
  if (!applyTo(associatedCands_)) applyTo(unassignedCands_);
  if (!bridgeable) return;

  store_.setOverride(key, sTopic, checked);

  associatedView_ = toVariantList(associatedCands_);
  unassignedView_ = toVariantList(unassignedCands_);
  emit topicsChanged();

  rebuildSession();
}

void Ros2BridgeManagerGui::checkAllAssociated()
{
  const std::string key = currentKey();
  for (auto &c : associatedCands_)
  {
    if (c.bridgeable)
    {
      c.checked = true;
      store_.setOverride(key, c.gzTopic, true);
    }
  }
  associatedView_ = toVariantList(associatedCands_);
  emit topicsChanged();
  rebuildSession();
}

void Ros2BridgeManagerGui::uncheckAllCurrentModel()
{
  const std::string key = currentKey();
  auto unset = [&](std::vector<BridgeTopicCandidate> &cs)
  {
    for (auto &c : cs)
    {
      if (!c.bridgeable) continue;
      c.checked = false;
      store_.setOverride(key, c.gzTopic, false);
    }
  };
  unset(associatedCands_);
  unset(unassignedCands_);

  associatedView_ = toVariantList(associatedCands_);
  unassignedView_ = toVariantList(unassignedCands_);
  emit topicsChanged();
  rebuildSession();
}

void Ros2BridgeManagerGui::resetCurrentModelSelection()
{
  store_.resetKey(currentKey());
  recomputeAndPublish();
}

void Ros2BridgeManagerGui::setIncludeAllModels(bool enabled)
{
  if (includeAllModels_ == enabled) return;
  includeAllModels_ = enabled;
  emit includeAllModelsChanged();
  rebuildSession();
}

void Ros2BridgeManagerGui::setAutoRefresh(bool enabled)
{
  if (autoRefresh_ == enabled) return;
  autoRefresh_ = enabled;
  if (autoRefresh_)
    autoRefreshTimer_.start();
  else
    autoRefreshTimer_.stop();
  emit autoRefreshChanged();
}

void Ros2BridgeManagerGui::onAutoRefreshTick()
{
  if (busy_) return;
  refresh();
}

void Ros2BridgeManagerGui::copyBridgeCommand()
{
  if (auto *clip = QGuiApplication::clipboard())
    clip->setText(bridgeCommand_);
}

}  // namespace gz_ros2_bridge_manager

GZ_ADD_PLUGIN(gz_ros2_bridge_manager::Ros2BridgeManagerGui,
              gz::gui::Plugin)
