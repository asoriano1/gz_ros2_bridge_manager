#include <gtest/gtest.h>

#include "gz_ros2_bridge_manager/BridgeTypeMapper.hh"
#include "gz_ros2_bridge_manager/GazeboTopicDiscovery.hh"
#include "gz_ros2_bridge_manager/TopicAssociationHeuristic.hh"

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

}  // namespace

// ---- Helper: sanitization --------------------------------------------------

TEST(SanitizeName, BasicCases)
{
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName("rbvogui_1"), "rbvogui_1");
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName("RBVogui-1"), "rbvogui_1");
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName("rb-vogui_1.0"), "rb_vogui_1_0");
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName("__foo__bar__"), "foo_bar");
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName(""), "");
  EXPECT_EQ(TopicAssociationHeuristic::sanitizeName("---"), "");
}

// ---- Helper: tokenContains -------------------------------------------------

TEST(TokenContains, RespectsIdentifierBoundaries)
{
  EXPECT_TRUE(TopicAssociationHeuristic::tokenContains("/rbvogui/scan", "rbvogui"));
  EXPECT_TRUE(TopicAssociationHeuristic::tokenContains("/foo/bar/rbvogui", "rbvogui"));
  EXPECT_TRUE(TopicAssociationHeuristic::tokenContains("rbvogui_1/scan", "rbvogui_1"));

  // "_" continues the identifier — "rbvogui" must NOT match "rbvogui_1"
  EXPECT_FALSE(TopicAssociationHeuristic::tokenContains("/rbvogui_1/scan", "rbvogui"));
  EXPECT_FALSE(TopicAssociationHeuristic::tokenContains("/foo_rbvogui/scan", "rbvogui"));

  // Empty / oversized
  EXPECT_FALSE(TopicAssociationHeuristic::tokenContains("/scan", ""));
  EXPECT_FALSE(TopicAssociationHeuristic::tokenContains("", "foo"));
  EXPECT_FALSE(TopicAssociationHeuristic::tokenContains("foo", "foobar"));
}

// ---- Helper: generic detection ---------------------------------------------

TEST(IsGeneric, KnownGenericTopics)
{
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/clock"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/scan"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/imu"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/imu/data"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/tf"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/tf_static"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/joint_states"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/camera/image"));
  EXPECT_TRUE(TopicAssociationHeuristic::isGenericTopic("/depth/image"));

  // Model-scoped paths are NOT generic.
  EXPECT_FALSE(TopicAssociationHeuristic::isGenericTopic("/model/r1/scan"));
  EXPECT_FALSE(TopicAssociationHeuristic::isGenericTopic("/r1/scan"));
  EXPECT_FALSE(TopicAssociationHeuristic::isGenericTopic("/world/default/model/r1/scan"));
}

// ============================================================================
// Spec test 1 — Exact /model/<name>/ path → associated, checked
// ============================================================================
TEST(Heuristic, ExactModelPathMatch)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/world/default/model/rbvogui_1/link/lidar/sensor/scan/scan",
              "gz.msgs.LaserScan", mapper),
  };

  auto r = heur.associate("rbvogui_1", "default", topics, {"rbvogui_1"});
  ASSERT_EQ(r.associated.size(), 1u);
  EXPECT_EQ(r.associated[0].category, AssociationCategory::ExactModelPath);
  EXPECT_TRUE(r.associated[0].checked);
  EXPECT_FALSE(r.associated[0].ambiguous);
  EXPECT_TRUE(r.unassigned.empty());
}

// ============================================================================
// Spec test 2 — Direct substring match → associated, checked
// ============================================================================
TEST(Heuristic, DirectSubstringMatch)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/rbvogui_1/scan", "gz.msgs.LaserScan", mapper),
  };

  auto r = heur.associate("rbvogui_1", "default", topics, {"rbvogui_1"});
  ASSERT_EQ(r.associated.size(), 1u);
  EXPECT_EQ(r.associated[0].category, AssociationCategory::ContainsModelName);
  EXPECT_TRUE(r.associated[0].checked);
}

// ============================================================================
// Spec test 3 — Generic /scan stays unassigned even with a model selected
// ============================================================================
TEST(Heuristic, GenericTopicStaysUnassigned)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/scan", "gz.msgs.LaserScan", mapper),
  };

  auto r = heur.associate("rbvogui_1", "default", topics, {"rbvogui_1"});
  EXPECT_TRUE(r.associated.empty());
  ASSERT_EQ(r.unassigned.size(), 1u);
  EXPECT_FALSE(r.unassigned[0].checked);
  EXPECT_TRUE(r.unassigned[0].isGeneric);
  EXPECT_EQ(r.unassigned[0].category, AssociationCategory::CompatibleButUnassigned);
}

// ============================================================================
// Spec test 4 — /clock stays global / unassigned, has explanation
// ============================================================================
TEST(Heuristic, ClockIsGlobalAndUnassigned)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/clock", "gz.msgs.Clock", mapper),
  };

  // With or without a selected model, /clock must not be auto-checked.
  auto r1 = heur.associate("rbvogui_1", "default", topics, {"rbvogui_1"});
  ASSERT_EQ(r1.unassigned.size(), 1u);
  EXPECT_FALSE(r1.unassigned[0].checked);
  EXPECT_TRUE(r1.unassigned[0].isGeneric);
  EXPECT_FALSE(r1.unassigned[0].warning.empty()) << "Should have a warning about /clock";

  auto r2 = heur.associate("", "default", topics, {});
  ASSERT_EQ(r2.unassigned.size(), 1u);
  EXPECT_FALSE(r2.unassigned[0].checked);
}

// ============================================================================
// Spec test 5 — Ambiguous topic with similar model names
// models {rbvogui_1, rbvogui}, selected = rbvogui, topic = /rbvogui_1/scan
// → must NOT auto-check for rbvogui (ideally ambiguous or unassigned)
// ============================================================================
TEST(Heuristic, AmbiguousNotAutoCheckedForShorterModel)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/rbvogui_1/scan", "gz.msgs.LaserScan", mapper),
  };
  std::vector<std::string> allModels{"rbvogui_1", "rbvogui"};

  // For the shorter "rbvogui": tokenContains rejects the substring (because
  // the next char "_" continues the identifier), so it lands unassigned —
  // either way, it must NOT be auto-checked.
  auto r = heur.associate("rbvogui", "default", topics, allModels);
  EXPECT_TRUE(r.associated.empty()) << "rbvogui must not own /rbvogui_1/scan";
  ASSERT_EQ(r.unassigned.size(), 1u);
  EXPECT_FALSE(r.unassigned[0].checked);
}

TEST(Heuristic, LongerModelStillOwnsItsTopic)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/rbvogui_1/scan", "gz.msgs.LaserScan", mapper),
  };
  std::vector<std::string> allModels{"rbvogui_1", "rbvogui"};

  // rbvogui is shorter, so when the user picks rbvogui_1 the heuristic
  // should still associate (no ambiguity since "rbvogui" can't token-match).
  auto r = heur.associate("rbvogui_1", "default", topics, allModels);
  ASSERT_EQ(r.associated.size(), 1u);
  EXPECT_TRUE(r.associated[0].checked);
  EXPECT_FALSE(r.associated[0].ambiguous);
}

// ============================================================================
// Spec test 6 — Unsupported gz type
// ============================================================================
TEST(Heuristic, UnsupportedGzTypeGoesToUnsupported)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/r1/some_proprietary", "gz.msgs.NonExistentType", mapper),
  };

  auto r = heur.associate("r1", "default", topics, {"r1"});
  EXPECT_TRUE(r.associated.empty());
  EXPECT_TRUE(r.unassigned.empty());
  ASSERT_EQ(r.unsupported.size(), 1u);
  EXPECT_EQ(r.unsupported[0].category, AssociationCategory::Unsupported);
  EXPECT_FALSE(r.unsupported[0].bridgeable);
}

// Pose_V is in the supported list (mapped to tf2_msgs/TFMessage), so use a
// truly unsupported type for the negative case above. Pose_V here documents
// that intentionally bridgeable types still show up in associated/unassigned.
TEST(Heuristic, PoseVIsBridgeable)
{
  BridgeTypeMapper mapper;
  EXPECT_TRUE(mapper.isBridgeable("gz.msgs.Pose_V"));
}

// ============================================================================
// Extra: empty selection puts everything bridgeable into "unassigned"
// ============================================================================
TEST(Heuristic, EmptySelectionUnassignsAllBridgeable)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/r1/scan", "gz.msgs.LaserScan", mapper),
    makeEntry("/clock",   "gz.msgs.Clock",     mapper),
    makeEntry("/foo",     "gz.msgs.NoSuchType", mapper),
  };

  auto r = heur.associate("", "default", topics, {"r1"});
  EXPECT_TRUE(r.associated.empty());
  EXPECT_EQ(r.unassigned.size(), 2u);
  EXPECT_EQ(r.unsupported.size(), 1u);
  for (const auto &c : r.unassigned)
    EXPECT_FALSE(c.checked);
}

// ============================================================================
// Extra: a Gazebo-scoped path that looks like /world/<w>/model/<m>/...
// ============================================================================
TEST(Heuristic, WorldScopedModelPathMatch)
{
  BridgeTypeMapper mapper;
  TopicAssociationHeuristic heur;

  std::vector<GzTopicEntry> topics{
    makeEntry("/world/default/model/turtle/joint_state",
              "gz.msgs.Model", mapper),  // unsupported type — for path test
    makeEntry("/world/default/model/turtle/link/base/sensor/imu/imu",
              "gz.msgs.IMU", mapper),
  };

  auto r = heur.associate("turtle", "default", topics, {"turtle"});
  // Bridgeable IMU should be associated via ExactModelPath
  ASSERT_EQ(r.associated.size(), 1u);
  EXPECT_EQ(r.associated[0].category, AssociationCategory::ExactModelPath);
  EXPECT_TRUE(r.associated[0].checked);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
