#include "gz_ros2_bridge_manager/BridgeSession.hh"

#include <algorithm>
#include <unordered_set>

#include "gz_ros2_bridge_manager/BridgeCommandBuilder.hh"

namespace gz_ros2_bridge_manager
{

namespace
{

void appendCheckedSpecs(const std::vector<BridgeTopicCandidate> &cs,
                        std::vector<std::string> &out,
                        std::unordered_set<std::string> &seen,
                        int &counter)
{
  for (const auto &c : cs)
  {
    if (!c.bridgeable || !c.checked || c.bridgeSpec.empty())
      continue;
    if (seen.insert(c.bridgeSpec).second)
    {
      out.push_back(c.bridgeSpec);
      ++counter;
    }
  }
}

}  // namespace

BridgeSessionResult BridgeSessionBuilder::build(
    const std::string &worldName,
    const std::string &currentKey,
    const std::vector<BridgeTopicCandidate> &currentCandidates,
    const ModelTopicSelectionStore &store,
    const TopicAssociationHeuristic &heuristic,
    const std::vector<GzTopicEntry> &discoveredTopics,
    const std::vector<std::string> &allModelsInWorld,
    bool includeAllModels)
{
  BridgeSessionResult result;
  std::unordered_set<std::string> seen;

  // ---- Current model contribution -----------------------------------------
  appendCheckedSpecs(currentCandidates, result.specs, seen,
                     result.currentModelChecked);

  // ---- Other model contributions (only when includeAllModels is on) -------
  if (includeAllModels)
  {
    for (const auto &key : store.allKeys())
    {
      if (key == currentKey) continue;
      if (ModelTopicSelectionStore::worldOfKey(key) != worldName) continue;

      const std::string modelName = ModelTopicSelectionStore::modelOfKey(key);

      auto r = heuristic.associate(modelName, worldName,
                                    discoveredTopics, allModelsInWorld);
      store.applyOverrides(key, r.associated);
      store.applyOverrides(key, r.unassigned);

      appendCheckedSpecs(r.associated, result.specs, seen,
                         result.otherModelsChecked);
      appendCheckedSpecs(r.unassigned, result.specs, seen,
                         result.otherModelsChecked);
    }
  }

  // ---- Missing topics ------------------------------------------------------
  // Topics the user explicitly checked under any in-scope key but which are
  // not in the current discovered set.
  std::unordered_set<std::string> discovered;
  discovered.reserve(discoveredTopics.size());
  for (const auto &t : discoveredTopics)
    discovered.insert(t.topicName);

  std::unordered_set<std::string> missingSet;
  auto collectMissingFor = [&](const std::string &k)
  {
    for (const auto &topic : store.manuallyChecked(k))
    {
      if (!discovered.count(topic))
        missingSet.insert(topic);
    }
  };

  collectMissingFor(currentKey);
  if (includeAllModels)
  {
    for (const auto &key : store.allKeys())
    {
      if (key == currentKey) continue;
      if (ModelTopicSelectionStore::worldOfKey(key) != worldName) continue;
      collectMissingFor(key);
    }
  }

  result.missingTopics.assign(missingSet.begin(), missingSet.end());
  std::sort(result.missingTopics.begin(), result.missingTopics.end());

  // ---- Commands ------------------------------------------------------------
  if (!result.specs.empty())
  {
    std::string cmd = "ros2 run ros_gz_bridge parameter_bridge";
    std::string wrapped = cmd;
    for (const auto &s : result.specs)
    {
      cmd.push_back(' ');
      cmd.append(s);
      wrapped.append(" \\\n  ");
      wrapped.append(s);
    }
    result.command        = std::move(cmd);
    result.commandWrapped = std::move(wrapped);
  }

  return result;
}

}  // namespace gz_ros2_bridge_manager
