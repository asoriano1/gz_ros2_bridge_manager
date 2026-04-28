#include <gtest/gtest.h>

#include "gz_ros2_bridge_manager/BridgeSession.hh"
#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

GzTopicEntry makeEntry(const std::string &topic, const std::string &gzType,
                       const BridgeTypeMapper &mapper)
{
  GzTopicEntry e;
  e.topicName   = topic;
  e.gzMsgType   = gzType;
  e.bridgeable  = mapper.isBridgeable(gzType);
  e.ros2MsgType = mapper.ros2Type(gzType);
  e.bridgeSpec  = mapper.bridgeSpec(topic, gzType);
  return e;
}

// Helper: full pipeline used by the plugin to compute candidates for a model
std::vector<BridgeTopicCandidate> currentCandsFor(
    const std::string &model, const std::string &world,
    const std::vector<GzTopicEntry> &topics,
    const std::vector<std::string> &allModels,
    const ModelTopicSelectionStore &store)
{
  TopicAssociationHeuristic heur;
  auto r = heur.associate(model, world, topics, allModels);
  const std::string key = model.empty()
      ? ModelTopicSelectionStore::manualKey(world)
      : ModelTopicSelectionStore::keyForModel(world, model);
  store.applyOverrides(key, r.associated);
  store.applyOverrides(key, r.unassigned);

  std::vector<BridgeTopicCandidate> all;
  all.insert(all.end(), r.associated.begin(), r.associated.end());
  all.insert(all.end(), r.unassigned.begin(), r.unassigned.end());
  return all;
}

}  // namespace

// ============================================================================
// Spec test 4 — Missing previously selected topic is reported, not commanded
// ============================================================================
TEST(BridgeSession, MissingTopicReportedNotCommanded)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;

  // /scan was previously checked under model "r1" but is not advertised now.
  const std::string key = ModelTopicSelectionStore::keyForModel("default", "r1");
  store.setOverride(key, "/scan", true);

  std::vector<GzTopicEntry> discovered{
    makeEntry("/r1/imu/data", "gz.msgs.IMU", mapper),
  };
  auto cur = currentCandsFor("r1", "default", discovered, {"r1"}, store);

  auto session = BridgeSessionBuilder::build(
      "default", key, cur, store, heur, discovered, {"r1"},
      /*includeAllModels=*/false);

  // /scan is not in discovered, so it must not appear in the command.
  EXPECT_EQ(session.command.find("/scan"), std::string::npos);
  EXPECT_NE(session.command.find("/r1/imu/data"), std::string::npos);

  // …but it must be flagged as missing.
  ASSERT_EQ(session.missingTopics.size(), 1u);
  EXPECT_EQ(session.missingTopics[0], "/scan");
}

// ============================================================================
// Spec test 6 — IncludeAllModels unions checked topics from two models
// ============================================================================
TEST(BridgeSession, IncludeAllModelsUnionsAcrossModels)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;

  // User curated model A → checks /world/default/model/A/scan
  const std::string keyA = ModelTopicSelectionStore::keyForModel("default", "A");
  store.setOverride(keyA, "/world/default/model/A/link/lidar/scan", true);

  // Selected model is B; their topic is auto-checked by heuristic.
  const std::string keyB = ModelTopicSelectionStore::keyForModel("default", "B");

  std::vector<GzTopicEntry> discovered{
    makeEntry("/world/default/model/A/link/lidar/scan", "gz.msgs.LaserScan", mapper),
    makeEntry("/world/default/model/B/link/imu/imu",    "gz.msgs.IMU",       mapper),
  };
  std::vector<std::string> allModels{"A", "B"};

  auto curB = currentCandsFor("B", "default", discovered, allModels, store);

  // Without includeAllModels: only B's topic.
  auto noAll = BridgeSessionBuilder::build(
      "default", keyB, curB, store, heur, discovered, allModels,
      /*includeAllModels=*/false);
  EXPECT_NE(noAll.command.find("/world/default/model/B/link/imu/imu"),
            std::string::npos);
  EXPECT_EQ(noAll.command.find("/world/default/model/A/link/lidar/scan"),
            std::string::npos);
  EXPECT_EQ(noAll.otherModelsChecked, 0);

  // With includeAllModels: both A and B contribute.
  auto withAll = BridgeSessionBuilder::build(
      "default", keyB, curB, store, heur, discovered, allModels,
      /*includeAllModels=*/true);
  EXPECT_NE(withAll.command.find("/world/default/model/A/link/lidar/scan"),
            std::string::npos);
  EXPECT_NE(withAll.command.find("/world/default/model/B/link/imu/imu"),
            std::string::npos);
  EXPECT_GE(withAll.otherModelsChecked, 1);
}

// ============================================================================
// Spec test 7 — IncludeAllModels deduplicates identical specs
// ============================================================================
TEST(BridgeSession, IncludeAllModelsDeduplicates)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;

  // Two models both manually checked the same global topic.
  const std::string keyA = ModelTopicSelectionStore::keyForModel("default", "A");
  const std::string keyB = ModelTopicSelectionStore::keyForModel("default", "B");
  store.setOverride(keyA, "/clock", true);
  store.setOverride(keyB, "/clock", true);

  std::vector<GzTopicEntry> discovered{
    makeEntry("/clock", "gz.msgs.Clock", mapper),
  };

  auto curA = currentCandsFor("A", "default", discovered, {"A", "B"}, store);
  auto session = BridgeSessionBuilder::build(
      "default", keyA, curA, store, heur, discovered, {"A", "B"},
      /*includeAllModels=*/true);

  // /clock spec must appear exactly once.
  const auto &cmd = session.command;
  size_t pos = cmd.find("/clock@");
  ASSERT_NE(pos, std::string::npos);
  size_t second = cmd.find("/clock@", pos + 1);
  EXPECT_EQ(second, std::string::npos) << "spec must be deduped";
  EXPECT_EQ(session.specs.size(), 1u);
}

// ============================================================================
// Spec test 8 — Reset model selection returns to heuristic defaults
// ============================================================================
TEST(BridgeSession, ResetKeyReturnsToHeuristicDefaults)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;
  const std::string key = ModelTopicSelectionStore::keyForModel("default", "r1");

  std::vector<GzTopicEntry> discovered{
    makeEntry("/r1/imu/data", "gz.msgs.IMU", mapper),  // associated, checked-by-default
  };

  // User unchecks the auto-associated topic.
  store.setOverride(key, "/r1/imu/data", false);
  auto cur1 = currentCandsFor("r1", "default", discovered, {"r1"}, store);
  auto s1 = BridgeSessionBuilder::build(
      "default", key, cur1, store, heur, discovered, {"r1"}, false);
  EXPECT_TRUE(s1.command.empty()) << "user override should remove the spec";

  // Reset → heuristic default returns
  store.resetKey(key);
  auto cur2 = currentCandsFor("r1", "default", discovered, {"r1"}, store);
  auto s2 = BridgeSessionBuilder::build(
      "default", key, cur2, store, heur, discovered, {"r1"}, false);
  EXPECT_NE(s2.command.find("/r1/imu/data"), std::string::npos);
}

// ============================================================================
// Manual mode is its own bucket (independent from any model key)
// ============================================================================
TEST(BridgeSession, ManualModeIsIndependentBucket)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;

  const std::string keyA   = ModelTopicSelectionStore::keyForModel("default", "A");
  const std::string keyMan = ModelTopicSelectionStore::manualKey("default");
  store.setOverride(keyMan, "/scan", true);

  std::vector<GzTopicEntry> discovered{
    makeEntry("/scan", "gz.msgs.LaserScan", mapper),
  };

  // In model A view, /scan is generic → unchecked.
  auto curA = currentCandsFor("A", "default", discovered, {"A"}, store);
  auto sA = BridgeSessionBuilder::build(
      "default", keyA, curA, store, heur, discovered, {"A"}, false);
  EXPECT_TRUE(sA.command.empty()) << "manual override must not leak into A";

  // In manual view, /scan is checked.
  auto curM = currentCandsFor("", "default", discovered, {"A"}, store);
  auto sM = BridgeSessionBuilder::build(
      "default", keyMan, curM, store, heur, discovered, {"A"}, false);
  EXPECT_NE(sM.command.find("/scan"), std::string::npos);
}

// ============================================================================
// Refresh re-applies stored overrides
// ============================================================================
TEST(BridgeSession, RefreshReappliesOverrides)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;
  const std::string key = ModelTopicSelectionStore::keyForModel("default", "r1");

  // First "discovery": nothing yet.
  std::vector<GzTopicEntry> empty;
  auto cur0 = currentCandsFor("r1", "default", empty, {"r1"}, store);
  auto s0 = BridgeSessionBuilder::build("default", key, cur0, store, heur, empty, {"r1"}, false);
  EXPECT_TRUE(s0.command.empty());

  // User selects a topic that is currently missing.
  store.setOverride(key, "/r1/scan", true);
  auto s1 = BridgeSessionBuilder::build("default", key, cur0, store, heur, empty, {"r1"}, false);
  EXPECT_TRUE(s1.command.empty());
  EXPECT_EQ(s1.missingTopics.size(), 1u);

  // Second "discovery" finds it — the stored override should pull it in.
  std::vector<GzTopicEntry> later{ makeEntry("/r1/scan", "gz.msgs.LaserScan", mapper) };
  auto cur2 = currentCandsFor("r1", "default", later, {"r1"}, store);
  auto s2 = BridgeSessionBuilder::build("default", key, cur2, store, heur, later, {"r1"}, false);
  EXPECT_NE(s2.command.find("/r1/scan"), std::string::npos);
  EXPECT_TRUE(s2.missingTopics.empty());
}

// ============================================================================
// Counts: currentModelChecked vs otherModelsChecked
// ============================================================================
TEST(BridgeSession, ChecksAreCountedCorrectly)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;
  ModelTopicSelectionStore store;

  const std::string keyA = ModelTopicSelectionStore::keyForModel("default", "A");
  const std::string keyB = ModelTopicSelectionStore::keyForModel("default", "B");

  store.setOverride(keyB, "/B_topic", true);  // curated for B previously

  std::vector<GzTopicEntry> discovered{
    makeEntry("/world/default/model/A/imu", "gz.msgs.IMU",       mapper),
    makeEntry("/B_topic",                   "gz.msgs.LaserScan", mapper),
  };

  auto curA = currentCandsFor("A", "default", discovered, {"A", "B"}, store);

  auto withAll = BridgeSessionBuilder::build(
      "default", keyA, curA, store, heur, discovered, {"A", "B"}, true);
  EXPECT_EQ(withAll.currentModelChecked, 1);
  EXPECT_EQ(withAll.otherModelsChecked,  1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
