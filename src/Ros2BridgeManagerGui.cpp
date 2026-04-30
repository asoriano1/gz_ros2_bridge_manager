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

#include "gz_ros2_bridge_manager/BridgeCommandBuilder.hh"
#include "gz_ros2_bridge_manager/EcmSensorDiscovery.hh"
#include "gz_ros2_bridge_manager/EcmSensorExtractor.hh"
#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"  // isGenericTopic static helper
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

constexpr int  kAutoRefreshIntervalMs = 2500;
constexpr const char *kAdditionalMarker = "__additional__";

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

QVariantMap sensorToVariantMap(const DiscoveredSensor &ds)
{
  QVariantMap m;
  m[QStringLiteral("modelName")]     = QString::fromStdString(ds.sensor.modelName);
  m[QStringLiteral("linkName")]      = QString::fromStdString(ds.sensor.linkName);
  m[QStringLiteral("sensorName")]    = QString::fromStdString(ds.sensor.sensorName);
  m[QStringLiteral("sensorType")]    = QString::fromStdString(ds.sensor.sensorType);
  m[QStringLiteral("declaredTopic")] = QString::fromStdString(ds.sensor.declaredTopic);
  m[QStringLiteral("fallbackPrefix")]= QString::fromStdString(ds.sensor.fallbackGazeboTopicPrefix);
  m[QStringLiteral("matchSource")]   = QString::fromLatin1(matchSourceName(ds.matchSource));
  m[QStringLiteral("nestedModel")]   = ds.sensor.nestedModel;
  m[QStringLiteral("resolved")]      = ds.resolved;
  m[QStringLiteral("warning")]       = QString::fromStdString(ds.warning);
  m[QStringLiteral("topicCount")]    = static_cast<int>(ds.matchedTopicNames.size());

  // Per-topic detail entries — checked/bridgeable filled in buildModelCard.
  QVariantList topicDetails;
  for (size_t i = 0; i < ds.matchedTopicNames.size(); ++i)
  {
    QVariantMap td;
    td[QStringLiteral("topic")]      = QString::fromStdString(ds.matchedTopicNames[i]);
    td[QStringLiteral("checked")]    = false;   // overwritten in buildModelCard
    td[QStringLiteral("bridgeable")] = true;    // overwritten in buildModelCard

    QString gzType, ros2Type;
    if (i < ds.matchedBridgeSpecs.size())
    {
      const auto &spec = ds.matchedBridgeSpecs[i];
      const auto at1 = spec.find('@');
      if (at1 != std::string::npos)
      {
        const auto at2 = spec.find('@', at1 + 1);
        if (at2 != std::string::npos)
        {
          ros2Type = QString::fromStdString(spec.substr(at1 + 1, at2 - at1 - 1));
          gzType   = QString::fromStdString(spec.substr(at2 + 1));
        }
      }
    }
    td[QStringLiteral("gzType")]   = gzType;
    td[QStringLiteral("ros2Type")] = ros2Type;
    topicDetails.append(td);
  }
  m[QStringLiteral("matchedTopicDetails")] = topicDetails;

  QStringList topicNames;
  for (const auto &s : ds.matchedTopicNames)
    topicNames << QString::fromStdString(s);
  m[QStringLiteral("matchedTopics")] = topicNames;

  return m;
}

// Build BridgeTopicCandidate entries from ECM-matched sensor topics.
// Strong matches (ECM exact/prefix/standard) are checked by default.
// Weak matches (NameMatch, TypeCompatibleFallback) are unchecked by default.
std::vector<BridgeTopicCandidate> ecmToCandidates(
    const ModelSensorTree &tree,
    const BridgeTypeMapper & /*mapper*/,
    std::unordered_set<std::string> &coveredTopics)
{
  std::vector<BridgeTopicCandidate> result;
  std::unordered_set<std::string> seenSpecs;

  for (const auto &ds : tree.sensors)
  {
    // Strong match = auto-checked; weak match = unchecked by default.
    const bool strongMatch =
        (ds.matchSource == MatchSource::EcmSensorTopicExact  ||
         ds.matchSource == MatchSource::EcmSensorTopicPrefix ||
         ds.matchSource == MatchSource::EcmStandardPrefix);

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
      c.checked    = strongMatch;   // weak matches start unchecked
      c.category   = AssociationCategory::EcmConfirmed;
      c.confidenceLabel = std::string(matchSourceName(ds.matchSource));
      c.isGeneric  = false;
      c.warning    = ds.warning;

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
      result.push_back(std::move(c));
    }

    // Truly unresolved sensor: placeholder so the sensor row remains visible.
    if (!ds.resolved)
    {
      BridgeTopicCandidate c;
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

  connect(&bridgeProcess_, &BridgeProcessManager::bridgeRunningChanged,
          this, &Ros2BridgeManagerGui::bridgeRunningChanged);
  connect(&bridgeProcess_, &BridgeProcessManager::bridgeBusyChanged,
          this, &Ros2BridgeManagerGui::bridgeBusyChanged);
  connect(&bridgeProcess_, &BridgeProcessManager::bridgeStatusTextChanged,
          this, &Ros2BridgeManagerGui::bridgeStatusTextChanged);
  connect(&bridgeProcess_, &BridgeProcessManager::bridgeOutputChanged,
          this, &Ros2BridgeManagerGui::bridgeOutputChanged);
  connect(&bridgeProcess_, &BridgeProcessManager::bridgeRestartRequiredChanged,
          this, &Ros2BridgeManagerGui::bridgeRestartRequiredChanged);
  connect(&bridgeProcess_, &BridgeProcessManager::runningBridgeCommandChanged,
          this, &Ros2BridgeManagerGui::runningBridgeCommandChanged);
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

void Ros2BridgeManagerGui::setStatus(const QString &text)
{
  if (statusText_ == text) return;
  statusText_ = text;
  emit statusTextChanged();
}

// ============================================================================
// Build one model accordion card
// ============================================================================

QVariantMap Ros2BridgeManagerGui::buildModelCard(
    const std::string &modelName,
    const ModelSensorTree &tree,
    const std::vector<BridgeTopicCandidate> &cands)
{
  // topic → {checked, bridgeable} lookup for embedding state into sensor rows.
  std::unordered_map<std::string, std::pair<bool, bool>> topicState;
  for (const auto &c : cands)
    topicState[c.gzTopic] = {c.checked, c.bridgeable};

  QVariantMap card;
  card[QStringLiteral("modelName")] = QString::fromStdString(modelName);

  const bool ecmActive = !tree.sensors.empty();
  int unresolvedCount  = 0;
  for (const auto &ds : tree.sensors)
    if (!ds.resolved) ++unresolvedCount;

  card[QStringLiteral("ecmAvailable")]          = ecmActive;
  card[QStringLiteral("ecmSensorCount")]         = static_cast<int>(tree.sensors.size());
  card[QStringLiteral("unresolvedSensorCount")]  = unresolvedCount;

  // Sensor list with per-topic checked/bridgeable state embedded.
  QVariantList sensors;
  for (const auto &ds : tree.sensors)
  {
    QVariantMap sm = sensorToVariantMap(ds);
    QVariantList enriched;
    for (const auto &tdVar : sm[QStringLiteral("matchedTopicDetails")].toList())
    {
      QVariantMap td = tdVar.toMap();
      const std::string topic = td[QStringLiteral("topic")].toString().toStdString();
      auto it = topicState.find(topic);
      if (it != topicState.end())
      {
        td[QStringLiteral("checked")]    = it->second.first;
        td[QStringLiteral("bridgeable")] = it->second.second;
      }
      enriched.append(td);
    }
    sm[QStringLiteral("matchedTopicDetails")] = enriched;
    sensors.append(sm);
  }
  card[QStringLiteral("sensors")] = sensors;

  // Count selected topics.
  int selected = 0;
  for (const auto &c : cands)
    if (c.checked && c.bridgeable) ++selected;
  card[QStringLiteral("selectedTopicCount")] = selected;

  return card;
}

// ============================================================================
// Recompute pipeline — ECM-first, no per-model heuristic
// ============================================================================

void Ros2BridgeManagerGui::recomputeAndPublish()
{
  // 1. ECM snapshot (thread-safe copy).
  std::vector<EcmSensorEntry> ecmSensors;
  {
    std::lock_guard<std::mutex> lk(ecmMutex_);
    ecmSensors = ecmSensors_;
  }

  const std::string world = worldName_.toStdString();

  // 2. Per-model: ECM match + candidate building.
  perModelCands_.clear();
  perModelTrees_.clear();
  warnings_.clear();

  std::unordered_set<std::string> globalCovered;

  for (const auto &modelName : discoveredModels_)
  {
    ModelSensorTree tree = EcmTopicMatcher::matchAll(
        world, ecmSensors, modelName, discoveredTopics_);

    std::unordered_set<std::string> modelCovered;
    std::vector<BridgeTopicCandidate> modelCands =
        ecmToCandidates(tree, mapper_, modelCovered);
    globalCovered.insert(modelCovered.begin(), modelCovered.end());

    const std::string key = ModelTopicSelectionStore::keyForModel(world, modelName);
    store_.applyOverrides(key, modelCands);

    perModelCands_[modelName] = std::move(modelCands);
    perModelTrees_[modelName] = std::move(tree);
  }

  // 3. Additional bridgeable topics — all bridgeable topics not covered by any
  //    ECM sensor match (strong or weak).
  additionalCands_.clear();
  unsupportedCands_.clear();

  const std::string addKey = world + "::" + kAdditionalMarker;

  for (const auto &entry : discoveredTopics_)
  {
    if (globalCovered.count(entry.topicName) > 0)
      continue;

    if (entry.bridgeable && !entry.bridgeSpec.empty())
    {
      BridgeTopicCandidate c;
      c.gzTopic    = entry.topicName;
      c.gzType     = entry.gzMsgType;
      c.ros2Type   = entry.ros2MsgType;
      c.bridgeSpec = entry.bridgeSpec;
      c.bridgeable = true;
      c.checked    = false;  // additional topics unchecked by default
      c.category   = AssociationCategory::CompatibleButUnassigned;
      c.isGeneric  = TopicAssociationHeuristic::isGenericTopic(entry.topicName);
      additionalCands_.push_back(std::move(c));
    }
    else if (!entry.bridgeable)
    {
      BridgeTopicCandidate c;
      c.gzTopic    = entry.topicName;
      c.gzType     = entry.gzMsgType;
      c.bridgeable = false;
      c.checked    = false;
      c.category   = AssociationCategory::Unsupported;
      unsupportedCands_.push_back(std::move(c));
    }
  }

  store_.applyOverrides(addKey, additionalCands_);

  // 4. Rebuild QML views.
  rebuildModelCards();
  rebuildBridgeCommand();
}

void Ros2BridgeManagerGui::rebuildModelCards()
{
  modelCards_.clear();

  for (const auto &modelName : discoveredModels_)
  {
    static const ModelSensorTree emptyTree;
    static const std::vector<BridgeTopicCandidate> emptyCands;

    const auto treeIt = perModelTrees_.find(modelName);
    const auto candIt = perModelCands_.find(modelName);

    const ModelSensorTree &tree =
        (treeIt != perModelTrees_.end()) ? treeIt->second : emptyTree;
    const std::vector<BridgeTopicCandidate> &cands =
        (candIt != perModelCands_.end()) ? candIt->second : emptyCands;

    modelCards_.append(buildModelCard(modelName, tree, cands));
  }

  additionalView_ = QVariantList{};
  for (const auto &c : additionalCands_)
    additionalView_.append(toVariantMap(c));

  unsupportedView_ = QVariantList{};
  for (const auto &c : unsupportedCands_)
    unsupportedView_.append(toVariantMap(c));

  emit modelsChanged();
}

void Ros2BridgeManagerGui::rebuildBridgeCommand()
{
  // All candidates in discovery order: per-model then additional.
  std::vector<BridgeTopicCandidate> allCands;
  for (const auto &modelName : discoveredModels_)
  {
    auto it = perModelCands_.find(modelName);
    if (it != perModelCands_.end())
      for (const auto &c : it->second)
        allCands.push_back(c);
  }
  for (const auto &c : additionalCands_)
    allCands.push_back(c);

  const auto selectedSpecs = BridgeCommandBuilder::selectedSpecs(allCands);
  bridgeCommand_        = QString::fromStdString(BridgeCommandBuilder::buildCommand(allCands));
  bridgeCommandDisplay_ = QString::fromStdString(BridgeCommandBuilder::buildCommandWrapped(allCands));

  QStringList desiredSpecs;
  desiredSpecs.reserve(static_cast<int>(selectedSpecs.size()));
  for (const auto &spec : selectedSpecs)
    desiredSpecs.push_back(QString::fromStdString(spec));
  bridgeProcess_.setDesiredSpecs(desiredSpecs);

  int count = 0;
  for (const auto &c : allCands)
    if (c.checked && c.bridgeable) ++count;
  selectedBridgeTopicCount_ = count;

  emit bridgeCommandChanged();
}

// ============================================================================
// Refresh — discover world + topics, then recompute
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

        self->discoveredModels_ = worldInfo.modelNames;
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
                  .arg(static_cast<int>(worldInfo.modelNames.size()))
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
// Per-topic toggles
// ============================================================================

void Ros2BridgeManagerGui::setTopicChecked(const QString &modelName,
                                            const QString &topic,
                                            bool checked)
{
  const std::string model  = modelName.toStdString();
  const std::string sTopic = topic.toStdString();
  const std::string world  = worldName_.toStdString();
  const std::string key    = ModelTopicSelectionStore::keyForModel(world, model);

  auto it = perModelCands_.find(model);
  if (it == perModelCands_.end()) return;

  bool found = false;
  for (auto &c : it->second)
  {
    if (c.gzTopic == sTopic && c.bridgeable)
    {
      c.checked = checked;
      store_.setOverride(key, sTopic, checked);
      found = true;
      break;
    }
  }
  if (!found) return;

  rebuildModelCards();
  rebuildBridgeCommand();
}

void Ros2BridgeManagerGui::setAdditionalTopicChecked(const QString &topic, bool checked)
{
  const std::string sTopic = topic.toStdString();
  const std::string world  = worldName_.toStdString();
  const std::string addKey = world + "::" + kAdditionalMarker;

  bool found = false;
  for (auto &c : additionalCands_)
  {
    if (c.gzTopic == sTopic && c.bridgeable)
    {
      c.checked = checked;
      store_.setOverride(addKey, sTopic, checked);
      found = true;
      break;
    }
  }
  if (!found) return;

  additionalView_ = QVariantList{};
  for (const auto &c : additionalCands_)
    additionalView_.append(toVariantMap(c));
  emit modelsChanged();

  rebuildBridgeCommand();
}

void Ros2BridgeManagerGui::resetModelSelection(const QString &modelName)
{
  const std::string model = modelName.toStdString();
  const std::string world = worldName_.toStdString();
  store_.resetKey(ModelTopicSelectionStore::keyForModel(world, model));
  recomputeAndPublish();
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

void Ros2BridgeManagerGui::runBridge()
{
  bridgeProcess_.runBridge();
}

void Ros2BridgeManagerGui::stopBridge()
{
  bridgeProcess_.stopBridge();
}

void Ros2BridgeManagerGui::restartBridge()
{
  bridgeProcess_.restartBridge();
}

void Ros2BridgeManagerGui::clearBridgeOutput()
{
  bridgeProcess_.clearBridgeOutput();
}

}  // namespace gz_ros2_bridge_manager

GZ_ADD_PLUGIN(gz_ros2_bridge_manager::Ros2BridgeManagerGui,
              gz::gui::Plugin)
