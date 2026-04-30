#include "gz_ros2_bridge_manager/Ros2BridgeManagerGui.hh"

#include <QtConcurrent/QtConcurrentRun>
#include <QClipboard>
#include <QDateTime>
#include <QGuiApplication>
#include <QVariantMap>

#include <algorithm>
#include <cctype>
#include <sstream>

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
constexpr bool kDebugTopicAssignment = true;

void debugLog(const std::string &message)
{
  if (!kDebugTopicAssignment)
    return;
  gzdbg << "[gz_ros2_bridge_manager][debug] " << message << '\n';
}

std::string gzTypeFromBridgeSpec(const std::string &spec)
{
  const auto at1 = spec.find('@');
  if (at1 == std::string::npos)
    return {};
  const auto at2 = spec.find('@', at1 + 1);
  if (at2 == std::string::npos || at2 + 1 >= spec.size())
    return {};
  return spec.substr(at2 + 1);
}

std::string ros2TypeFromBridgeSpec(const std::string &spec)
{
  const auto at1 = spec.find('@');
  if (at1 == std::string::npos)
    return {};
  const auto at2 = spec.find('@', at1 + 1);
  if (at2 == std::string::npos || at2 <= at1 + 1)
    return {};
  return spec.substr(at1 + 1, at2 - at1 - 1);
}

std::string normalizeBridgeSpecTopic(const std::string &spec)
{
  const auto at1 = spec.find('@');
  if (at1 == std::string::npos)
    return spec;
  return EcmTopicMatcher::normalizeTopic(spec.substr(0, at1)) + spec.substr(at1);
}

std::string matchedTopicSourceLabel(const DiscoveredSensor &ds,
                                    const std::string &topic,
                                    const std::string &gzType)
{
  const std::string normalizedTopic = EcmTopicMatcher::normalizeTopic(topic);
  const std::string normalizedDeclared =
      EcmTopicMatcher::normalizeTopic(ds.sensor.declaredTopic);

  if (!normalizedDeclared.empty() && normalizedTopic == normalizedDeclared)
  {
    if (ds.typeSource == "advertised")
      return "TopicInfo";
    if (ds.typeSource == "type inferred")
      return "inferred from sensorType";
    return "ECM SensorTopic";
  }

  if (gzType == "gz.msgs.CameraInfo")
    return "derived camera_info";

  if (ds.matchSource == MatchSource::EcmSensorTopicExact ||
      ds.matchSource == MatchSource::EcmSensorTopicPrefix)
  {
    return "ECM SensorTopic";
  }

  if (ds.matchSource == MatchSource::EcmStandardPrefix)
    return "other";

  return "other";
}

bool containsCameraOrInfo(const std::string &value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
  return lowered.find("camera") != std::string::npos ||
         lowered.find("info") != std::string::npos;
}

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
  m[QStringLiteral("topicListed")]   = ds.topicListed;
  m[QStringLiteral("topicInfoGzType")] = QString::fromStdString(ds.topicInfoGzType);
  m[QStringLiteral("inferredGzType")]  = QString::fromStdString(ds.inferredGzType);
  m[QStringLiteral("typeSource")]      = QString::fromStdString(ds.typeSource);
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
// The main workflow is ECM-only: declared SensorTopic matches are checked by
// default, unresolved sensors stay visible via placeholder entries.
std::vector<BridgeTopicCandidate> ecmToCandidates(
    const ModelSensorTree &tree,
    const BridgeTypeMapper & /*mapper*/,
    std::unordered_set<std::string> &coveredTopics)
{
  std::vector<BridgeTopicCandidate> result;
  std::unordered_set<std::string> seenSpecs;

  for (const auto &ds : tree.sensors)
  {
    // ECM-declared SensorTopic matches are auto-checked by default.
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
      coveredTopics.insert(EcmTopicMatcher::normalizeTopic(topicName));

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

  const ModelSensorTree allSensorsTree = EcmTopicMatcher::matchAll(
      world, ecmSensors, "", discoveredTopics_);

  const std::unordered_set<std::string> claimedEcmTopics =
      EcmTopicMatcher::claimedTopicNames(allSensorsTree);

  if (kDebugTopicAssignment)
  {
    debugLog("claimed topics:");
    for (const auto &topic : claimedEcmTopics)
      debugLog("  claimed: " + topic);

    debugLog("discovered camera/info topics:");
    for (const auto &entry : discoveredTopics_)
    {
      if (!containsCameraOrInfo(entry.topicName) &&
          !containsCameraOrInfo(entry.gzMsgType))
      {
        continue;
      }

      std::ostringstream oss;
      oss << "  discovered: topic=" << EcmTopicMatcher::normalizeTopic(entry.topicName)
          << ", gzType=" << (entry.gzMsgType.empty() ? "<none>" : entry.gzMsgType)
          << ", rosType=" << (entry.ros2MsgType.empty() ? "<none>" : entry.ros2MsgType)
          << ", bridgeable=" << (entry.bridgeable ? "true" : "false");
      debugLog(oss.str());
    }
  }

  std::unordered_map<std::string, ModelSensorTree> treesByModel;
  for (const auto &modelName : discoveredModels_)
  {
    ModelSensorTree tree;
    tree.worldName = world;
    tree.modelName = modelName;
    tree.ecmConfirmed = true;
    treesByModel.emplace(modelName, std::move(tree));
  }

  for (const auto &sensor : allSensorsTree.sensors)
  {
    auto &tree = treesByModel[sensor.sensor.modelName];
    if (tree.worldName.empty())
      tree.worldName = world;
    if (tree.modelName.empty())
      tree.modelName = sensor.sensor.modelName;
    tree.ecmConfirmed = true;
    tree.sensors.push_back(sensor);
  }

  for (const auto &modelName : discoveredModels_)
  {
    static const ModelSensorTree emptyTree;
    const auto treeIt = treesByModel.find(modelName);
    const ModelSensorTree &tree =
        (treeIt != treesByModel.end()) ? treeIt->second : emptyTree;

    std::unordered_set<std::string> modelCovered;
    std::vector<BridgeTopicCandidate> modelCands =
        ecmToCandidates(tree, mapper_, modelCovered);

    const std::string key = ModelTopicSelectionStore::keyForModel(world, modelName);
    store_.applyOverrides(key, modelCands);

    perModelCands_[modelName] = std::move(modelCands);
    perModelTrees_[modelName] = std::move(tree);
  }

  // 3. Additional bridgeable topics — all bridgeable topics not covered by
  //    any ECM SensorTopic match.
  additionalCands_.clear();
  unsupportedCands_.clear();

  const std::string addKey = world + "::" + kAdditionalMarker;

  const auto additionalBridgeableTopics =
      EcmTopicMatcher::bridgeableTopicsExcludingClaims(discoveredTopics_, allSensorsTree);

  if (kDebugTopicAssignment)
  {
    for (const auto &modelEntry : treesByModel)
    {
      const auto &tree = modelEntry.second;
      for (const auto &sensor : tree.sensors)
      {
        std::ostringstream header;
        header << "model=" << sensor.sensor.modelName
               << "(" << sensor.sensor.modelEntity << ")"
               << ", link=" << (sensor.sensor.linkName.empty() ? "<none>" : sensor.sensor.linkName)
               << "(" << sensor.sensor.linkEntity << ")"
               << ", sensor=" << sensor.sensor.sensorName
               << "(" << sensor.sensor.sensorEntity << ")"
               << ", sensorType=" << sensor.sensor.sensorType
               << ", SensorTopic="
               << (sensor.sensor.declaredTopic.empty() ? "<none>" : sensor.sensor.declaredTopic)
               << ", fallback="
               << (sensor.sensor.fallbackGazeboTopicPrefix.empty()
                       ? "<none>"
                       : sensor.sensor.fallbackGazeboTopicPrefix);
        debugLog(header.str());

        for (size_t i = 0; i < sensor.matchedTopicNames.size(); ++i)
        {
          const std::string topic =
              EcmTopicMatcher::normalizeTopic(sensor.matchedTopicNames[i]);
          const std::string spec =
              i < sensor.matchedBridgeSpecs.size() ? sensor.matchedBridgeSpecs[i] : std::string{};
          const std::string gzType = gzTypeFromBridgeSpec(spec);
          const std::string rosType = ros2TypeFromBridgeSpec(spec);
          const std::string source = matchedTopicSourceLabel(sensor, topic, gzType);

          std::ostringstream topicLog;
          topicLog << "  matched: topic=" << topic
                   << ", gzType=" << (gzType.empty() ? "<none>" : gzType)
                   << ", rosType=" << (rosType.empty() ? "<none>" : rosType)
                   << ", source=" << source;
          debugLog(topicLog.str());
        }
      }
    }

    debugLog("additional topics before dedup:");
    for (const auto &entry : discoveredTopics_)
    {
      if (!entry.bridgeable || entry.bridgeSpec.empty())
        continue;
      debugLog("  additional-pre: " + EcmTopicMatcher::normalizeTopic(entry.topicName));
    }

    debugLog("additional topics after dedup:");
    for (const auto &entry : additionalBridgeableTopics)
      debugLog("  additional-post: " + EcmTopicMatcher::normalizeTopic(entry.topicName));
  }

  for (const auto &entry : additionalBridgeableTopics)
  {
    BridgeTopicCandidate c;
    c.gzTopic    = EcmTopicMatcher::normalizeTopic(entry.topicName);
    c.gzType     = entry.gzMsgType;
    c.ros2Type   = entry.ros2MsgType;
    c.bridgeSpec = normalizeBridgeSpecTopic(entry.bridgeSpec);
    c.bridgeable = true;
    c.checked    = false;  // additional topics unchecked by default
    c.category   = AssociationCategory::CompatibleButUnassigned;
    c.isGeneric  = TopicAssociationHeuristic::isGenericTopic(entry.topicName);
    additionalCands_.push_back(std::move(c));
  }

  for (const auto &entry : discoveredTopics_)
  {
    if (claimedEcmTopics.count(EcmTopicMatcher::normalizeTopic(entry.topicName)) > 0)
      continue;
    if (!entry.bridgeable)
    {
      BridgeTopicCandidate c;
      c.gzTopic    = EcmTopicMatcher::normalizeTopic(entry.topicName);
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
