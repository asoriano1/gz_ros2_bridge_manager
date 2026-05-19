#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"

namespace gz_ros2_bridge_manager
{

// ---- Key construction & decomposition --------------------------------------

std::string ModelTopicSelectionStore::keyForModel(const std::string &world,
                                                   const std::string &model)
{
  return world + kKeySeparator + model;
}

std::string ModelTopicSelectionStore::manualKey(const std::string &world)
{
  return world + kKeySeparator + kManualMarker;
}

bool ModelTopicSelectionStore::isManualKey(const std::string &key)
{
  const std::string suffix = std::string(kKeySeparator) + kManualMarker;
  return key.size() >= suffix.size() &&
         key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string ModelTopicSelectionStore::worldOfKey(const std::string &key)
{
  const auto pos = key.find(kKeySeparator);
  if (pos == std::string::npos) return {};
  return key.substr(0, pos);
}

std::string ModelTopicSelectionStore::modelOfKey(const std::string &key)
{
  const auto pos = key.find(kKeySeparator);
  if (pos == std::string::npos) return {};
  const std::string tail = key.substr(pos + std::string(kKeySeparator).size());
  return tail == kManualMarker ? std::string{} : tail;
}

// ---- Override management ---------------------------------------------------

void ModelTopicSelectionStore::setOverride(const std::string &key,
                                           const std::string &topic,
                                           bool checked)
{
  auto &state = data_[key];
  if (checked)
  {
    state.unchecked.erase(topic);
    state.checked.insert(topic);
  }
  else
  {
    state.checked.erase(topic);
    state.unchecked.insert(topic);
  }
}

void ModelTopicSelectionStore::clearOverride(const std::string &key,
                                             const std::string &topic)
{
  auto it = data_.find(key);
  if (it == data_.end()) return;
  it->second.checked.erase(topic);
  it->second.unchecked.erase(topic);
  if (it->second.empty())
    data_.erase(it);
}

void ModelTopicSelectionStore::resetKey(const std::string &key)
{
  data_.erase(key);
}

// ---- Application -----------------------------------------------------------

void ModelTopicSelectionStore::applyOverrides(
    const std::string &key,
    std::vector<BridgeTopicCandidate> &candidates) const
{
  auto it = data_.find(key);
  if (it == data_.end()) return;
  const auto &state = it->second;

  for (auto &c : candidates)
  {
    if (!c.bridgeable) continue;
    if (state.checked.count(c.gzTopic))
    {
      c.checked = true;
    }
    else if (state.unchecked.count(c.gzTopic))
    {
      c.checked = false;
    }
  }
}

// ---- Inspection ------------------------------------------------------------

bool ModelTopicSelectionStore::hasKey(const std::string &key) const
{
  return data_.count(key) > 0;
}

bool ModelTopicSelectionStore::isManuallyChecked(
    const std::string &key, const std::string &topic) const
{
  auto it = data_.find(key);
  return it != data_.end() && it->second.checked.count(topic) > 0;
}

bool ModelTopicSelectionStore::isManuallyUnchecked(
    const std::string &key, const std::string &topic) const
{
  auto it = data_.find(key);
  return it != data_.end() && it->second.unchecked.count(topic) > 0;
}

bool ModelTopicSelectionStore::hasAnyOverride(
    const std::string &key, const std::string &topic) const
{
  return isManuallyChecked(key, topic) || isManuallyUnchecked(key, topic);
}

std::vector<std::string> ModelTopicSelectionStore::allKeys() const
{
  std::vector<std::string> keys;
  keys.reserve(data_.size());
  for (const auto &kv : data_)
    keys.push_back(kv.first);
  return keys;
}

std::unordered_set<std::string>
ModelTopicSelectionStore::manuallyChecked(const std::string &key) const
{
  auto it = data_.find(key);
  return it != data_.end() ? it->second.checked : std::unordered_set<std::string>{};
}

std::unordered_set<std::string>
ModelTopicSelectionStore::manuallyUnchecked(const std::string &key) const
{
  auto it = data_.find(key);
  return it != data_.end() ? it->second.unchecked : std::unordered_set<std::string>{};
}

}  // namespace gz_ros2_bridge_manager
