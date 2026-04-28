#pragma once

#include <string>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"
#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"
#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"

namespace gz_ros2_bridge_manager
{

struct BridgeSessionResult
{
  std::vector<std::string> specs;            // dedup-ordered bridge specs
  std::string command;                       // single-line ros2 run …
  std::string commandWrapped;                // multi-line, line-continued
  std::vector<std::string> missingTopics;    // checked-by-user but not advertised now
  int currentModelChecked = 0;
  int otherModelsChecked  = 0;               // additional from other models
};

// Pure function: builds a bridge session command from store + heuristic.
//
// `currentCandidates` is the candidate list for the *currently selected*
// model with overrides already applied — i.e. its `checked` field is the
// effective state.  We pass it in instead of re-running the heuristic so the
// caller's view stays consistent.
//
// When `includeAllModels` is true the result also unions checked topics
// from every *other* key in `store` whose world matches `worldName`.
// For each such key we re-run the heuristic for that key's model and apply
// that key's overrides — a model the user previously curated still
// contributes its checked topics.
//
// `missingTopics` lists explicitly-checked topics that are not present in
// `discoveredTopics` (across all keys considered).
class BridgeSessionBuilder
{
public:
  static BridgeSessionResult build(
      const std::string &worldName,
      const std::string &currentKey,
      const std::vector<BridgeTopicCandidate> &currentCandidates,
      const ModelTopicSelectionStore &store,
      const TopicAssociationHeuristic &heuristic,
      const std::vector<GzTopicEntry> &discoveredTopics,
      const std::vector<std::string> &allModelsInWorld,
      bool includeAllModels);
};

}  // namespace gz_ros2_bridge_manager
