#include "gz_ros2_bridge_manager/Ros2BridgeManagerGui.hh"

#include <QtConcurrent/QtConcurrentRun>
#include <QClipboard>
#include <QDateTime>
#include <QGuiApplication>
#include <QVariantMap>

#include <gz/gui/Application.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

#include "gz_ros2_bridge_manager/BridgeSession.hh"
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

}  // namespace

// ============================================================================
// Construction
// ============================================================================

Ros2BridgeManagerGui::Ros2BridgeManagerGui()
: gz::gui::Plugin()
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
  // Heuristic for current model
  auto result = heuristic_.associate(
      selectedModel_.toStdString(),
      worldName_.toStdString(),
      discoveredTopics_,
      discoveredModels_);

  // Apply current key's overrides
  const std::string key = currentKey();
  store_.applyOverrides(key, result.associated);
  store_.applyOverrides(key, result.unassigned);

  associatedCands_  = std::move(result.associated);
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
  currentCands.insert(currentCands.end(), associatedCands_.begin(), associatedCands_.end());
  currentCands.insert(currentCands.end(), unassignedCands_.begin(), unassignedCands_.end());

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
        // ---- World name ----
        const QString newWorld = QString::fromStdString(worldInfo.worldName);
        if (newWorld != self->worldName_)
        {
          self->worldName_ = newWorld;
          emit self->worldNameChanged();
        }

        // ---- Models ----
        QStringList modelNames;
        for (const auto &m : worldInfo.modelNames)
          modelNames << QString::fromStdString(m);

        if (modelNames != self->modelNames_)
        {
          self->modelNames_ = modelNames;
          emit self->modelNamesChanged();
        }
        self->discoveredModels_ = worldInfo.modelNames;

        // ---- Selected-model-disappeared handling ----
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
          // Clear stale warning if user re-selected a real model.
          self->modelGoneWarning_.clear();
          emit self->modelGoneWarningChanged();
        }

        self->discoveredTopics_ = topics;

        // ---- Status ----
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
  // The "model gone" warning is moot once a real model is picked again.
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
  const std::string key   = currentKey();
  const std::string sTopic = topic.toStdString();

  // Verify the topic is bridgeable in the current view; refuse otherwise.
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

  // Refresh the QML views without re-running the heuristic.
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
  if (busy_) return;  // skip if a refresh is already in flight
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
