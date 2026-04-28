#include <gtest/gtest.h>

#include <algorithm>

#include "gz_ros2_bridge_manager/ModelTopicSelectionStore.hh"

using namespace gz_ros2_bridge_manager;

namespace
{

BridgeTopicCandidate cand(const std::string &topic, bool bridgeable, bool checked)
{
  BridgeTopicCandidate c;
  c.gzTopic    = topic;
  c.bridgeable = bridgeable;
  c.checked    = checked;
  return c;
}

}  // namespace

// ---- Key helpers -----------------------------------------------------------

TEST(SelectionStoreKeys, ConstructionAndDecomposition)
{
  EXPECT_EQ(ModelTopicSelectionStore::keyForModel("default", "rbvogui_1"),
            "default::rbvogui_1");
  EXPECT_EQ(ModelTopicSelectionStore::manualKey("default"),
            "default::__manual__");

  EXPECT_TRUE (ModelTopicSelectionStore::isManualKey("default::__manual__"));
  EXPECT_FALSE(ModelTopicSelectionStore::isManualKey("default::rbvogui_1"));

  EXPECT_EQ(ModelTopicSelectionStore::worldOfKey("empty::rbvogui_1"), "empty");
  EXPECT_EQ(ModelTopicSelectionStore::modelOfKey("empty::rbvogui_1"), "rbvogui_1");
  EXPECT_EQ(ModelTopicSelectionStore::modelOfKey("empty::__manual__"), "");
  EXPECT_EQ(ModelTopicSelectionStore::worldOfKey("no_separator"), "");
}

// ---- Basic override management ---------------------------------------------

TEST(SelectionStore, EmptyStoreReportsNoOverrides)
{
  ModelTopicSelectionStore s;
  EXPECT_FALSE(s.hasKey("k"));
  EXPECT_FALSE(s.isManuallyChecked("k", "/x"));
  EXPECT_FALSE(s.isManuallyUnchecked("k", "/x"));
  EXPECT_TRUE (s.allKeys().empty());
}

TEST(SelectionStore, SetAndQueryOverrides)
{
  ModelTopicSelectionStore s;
  s.setOverride("default::r1", "/a", true);
  s.setOverride("default::r1", "/b", false);

  EXPECT_TRUE (s.isManuallyChecked  ("default::r1", "/a"));
  EXPECT_FALSE(s.isManuallyUnchecked("default::r1", "/a"));
  EXPECT_TRUE (s.isManuallyUnchecked("default::r1", "/b"));
  EXPECT_FALSE(s.isManuallyChecked  ("default::r1", "/b"));
  EXPECT_TRUE (s.hasAnyOverride     ("default::r1", "/a"));
}

TEST(SelectionStore, SetsAreMutuallyExclusive)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/a", true);
  s.setOverride("k", "/a", false);  // flip
  EXPECT_FALSE(s.isManuallyChecked  ("k", "/a"));
  EXPECT_TRUE (s.isManuallyUnchecked("k", "/a"));

  s.setOverride("k", "/a", true);   // flip back
  EXPECT_TRUE (s.isManuallyChecked  ("k", "/a"));
  EXPECT_FALSE(s.isManuallyUnchecked("k", "/a"));
}

TEST(SelectionStore, ClearOverrideRemovesFromBoth)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/a", true);
  s.clearOverride("k", "/a");
  EXPECT_FALSE(s.hasAnyOverride("k", "/a"));
  EXPECT_FALSE(s.hasKey("k")) << "Empty key should be removed";
}

TEST(SelectionStore, ResetKeyClearsAllOverrides)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/a", true);
  s.setOverride("k", "/b", false);
  s.resetKey("k");
  EXPECT_FALSE(s.hasKey("k"));
  EXPECT_FALSE(s.isManuallyChecked  ("k", "/a"));
  EXPECT_FALSE(s.isManuallyUnchecked("k", "/b"));
}

TEST(SelectionStore, KeysAreIndependent)
{
  ModelTopicSelectionStore s;
  s.setOverride("k1", "/a", true);
  s.setOverride("k2", "/a", false);
  EXPECT_TRUE (s.isManuallyChecked  ("k1", "/a"));
  EXPECT_TRUE (s.isManuallyUnchecked("k2", "/a"));
}

TEST(SelectionStore, AllKeysListsKnownKeys)
{
  ModelTopicSelectionStore s;
  s.setOverride("k1", "/a", true);
  s.setOverride("k2", "/b", true);
  auto keys = s.allKeys();
  std::sort(keys.begin(), keys.end());
  ASSERT_EQ(keys.size(), 2u);
  EXPECT_EQ(keys[0], "k1");
  EXPECT_EQ(keys[1], "k2");
}

// ---- applyOverrides --------------------------------------------------------

TEST(SelectionStore, ApplyOverridesMutatesCheckedField)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/a", true);   // explicitly checked
  s.setOverride("k", "/b", false);  // explicitly unchecked

  std::vector<BridgeTopicCandidate> cs{
    cand("/a", /*bridgeable=*/true,  /*checked=*/false),
    cand("/b", /*bridgeable=*/true,  /*checked=*/true),
    cand("/c", /*bridgeable=*/true,  /*checked=*/true),  // no override → unchanged
  };
  s.applyOverrides("k", cs);
  EXPECT_TRUE (cs[0].checked);  // overridden true
  EXPECT_FALSE(cs[1].checked);  // overridden false
  EXPECT_TRUE (cs[2].checked);  // untouched
}

TEST(SelectionStore, ApplyOverridesIgnoresUnbridgeable)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/x", true);
  std::vector<BridgeTopicCandidate> cs{ cand("/x", false, false) };
  s.applyOverrides("k", cs);
  EXPECT_FALSE(cs[0].checked) << "non-bridgeable must not be flipped";
}

TEST(SelectionStore, ApplyOverridesNoopForUnknownKey)
{
  ModelTopicSelectionStore s;
  std::vector<BridgeTopicCandidate> cs{
    cand("/a", true, true),
    cand("/b", true, false),
  };
  s.applyOverrides("nope", cs);
  EXPECT_TRUE (cs[0].checked);
  EXPECT_FALSE(cs[1].checked);
}

// ---- Spec scenario: manual check persists across selection round-trip ------

TEST(SelectionStore, ManualCheckPersistsOnRoundTrip)
{
  ModelTopicSelectionStore s;
  const std::string keyA = ModelTopicSelectionStore::keyForModel("w", "A");
  const std::string keyB = ModelTopicSelectionStore::keyForModel("w", "B");

  // User on A: manually checks /scan
  s.setOverride(keyA, "/scan", true);

  // Switch to B (heuristic + applyOverrides for B — should not touch A's set)
  std::vector<BridgeTopicCandidate> bCands{ cand("/scan", true, false) };
  s.applyOverrides(keyB, bCands);
  EXPECT_FALSE(bCands[0].checked) << "B must not inherit A's choice";

  // Switch back to A: heuristic produces /scan unchecked, store flips to true
  std::vector<BridgeTopicCandidate> aCands{ cand("/scan", true, false) };
  s.applyOverrides(keyA, aCands);
  EXPECT_TRUE(aCands[0].checked);
}

TEST(SelectionStore, ManualUncheckOfAutoCheckedPersists)
{
  ModelTopicSelectionStore s;
  const std::string key = ModelTopicSelectionStore::keyForModel("w", "A");
  s.setOverride(key, "/cam", false);

  // Heuristic auto-checks /cam (associated)
  std::vector<BridgeTopicCandidate> cs{ cand("/cam", true, true) };
  s.applyOverrides(key, cs);
  EXPECT_FALSE(cs[0].checked);
}

TEST(SelectionStore, ManualModeIsIndependentOfModelKeys)
{
  ModelTopicSelectionStore s;
  const std::string manual = ModelTopicSelectionStore::manualKey("w");
  const std::string keyA   = ModelTopicSelectionStore::keyForModel("w", "A");

  s.setOverride(manual, "/scan", true);

  std::vector<BridgeTopicCandidate> aCands{ cand("/scan", true, false) };
  s.applyOverrides(keyA, aCands);
  EXPECT_FALSE(aCands[0].checked) << "manual selection must not leak into A";

  std::vector<BridgeTopicCandidate> mCands{ cand("/scan", true, false) };
  s.applyOverrides(manual, mCands);
  EXPECT_TRUE(mCands[0].checked);
}

TEST(SelectionStore, ManuallyCheckedReturnsCorrectSet)
{
  ModelTopicSelectionStore s;
  s.setOverride("k", "/a", true);
  s.setOverride("k", "/b", true);
  s.setOverride("k", "/c", false);

  const auto checked = s.manuallyChecked("k");
  EXPECT_EQ(checked.size(), 2u);
  EXPECT_TRUE(checked.count("/a"));
  EXPECT_TRUE(checked.count("/b"));
  EXPECT_FALSE(checked.count("/c"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
