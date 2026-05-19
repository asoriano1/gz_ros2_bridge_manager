#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

namespace gz_ros2_bridge_manager
{

/// Per-model store of user-curated topic selections.
///
/// The store is keyed by a stable string built from world and model names:
///   - `"<world>::<model>"`    for a regular discovered model
///   - `"<world>::__manual__"` for the "no model" / manual selection mode
///
/// Each key carries two disjoint sets of explicit overrides:
///   - manuallyChecked   — the user explicitly enabled bridging for the topic
///   - manuallyUnchecked — the user explicitly disabled it
/// Topics that appear in neither set keep the heuristic's default state.
///
/// The class is pure C++ (no Qt, no Gazebo), which makes it trivially
/// unit-testable.
class ModelTopicSelectionStore
{
public:
  /// Marker used to build keys for the "manual" selection mode.
  static constexpr const char *kManualMarker = "__manual__";
  /// Separator used in `<world><sep><model>` style keys.
  static constexpr const char *kKeySeparator = "::";

  // ---- Key construction & decomposition ----------------------------------

  /// Builds the key for a real (world, model) pair.
  static std::string keyForModel(const std::string &world,
                                 const std::string &model);
  /// Builds the manual-mode key for a given world.
  static std::string manualKey(const std::string &world);
  /// Returns true iff `key` was produced by manualKey().
  static bool        isManualKey(const std::string &key);
  /// Returns the world component of the key, or "" if the separator is absent.
  static std::string worldOfKey(const std::string &key);
  /// Returns the model component of the key, or "" for manual-mode keys.
  static std::string modelOfKey(const std::string &key);

  // ---- Override management -----------------------------------------------

  /// Records an explicit user decision for `topic` under `key`. The checked
  /// and unchecked sets are kept disjoint: setting `checked=true` removes the
  /// topic from the unchecked set and vice-versa.
  void setOverride(const std::string &key, const std::string &topic,
                   bool checked);

  /// Removes `topic` from both override sets (the topic falls back to the
  /// heuristic default).
  void clearOverride(const std::string &key, const std::string &topic);

  /// Drops every override for `key`, returning the whole key to defaults.
  void resetKey(const std::string &key);

  // ---- Application --------------------------------------------------------

  /// For every candidate whose topic is recorded in the overrides for `key`,
  /// rewrites its `checked` flag accordingly. Non-bridgeable candidates are
  /// left untouched.
  void applyOverrides(const std::string &key,
                      std::vector<BridgeTopicCandidate> &candidates) const;

  // ---- Inspection ---------------------------------------------------------

  /// True iff at least one override has been recorded for `key`.
  bool hasKey(const std::string &key) const;
  /// True iff the user explicitly enabled `topic` under `key`.
  bool isManuallyChecked  (const std::string &key, const std::string &topic) const;
  /// True iff the user explicitly disabled `topic` under `key`.
  bool isManuallyUnchecked(const std::string &key, const std::string &topic) const;
  /// True iff the topic appears in either override set.
  bool hasAnyOverride     (const std::string &key, const std::string &topic) const;

  /// All keys for which at least one override exists.
  std::vector<std::string>          allKeys() const;
  /// All topics explicitly checked under `key`.
  std::unordered_set<std::string>   manuallyChecked  (const std::string &key) const;
  /// All topics explicitly unchecked under `key`.
  std::unordered_set<std::string>   manuallyUnchecked(const std::string &key) const;

private:
  struct PerKey
  {
    std::unordered_set<std::string> checked;
    std::unordered_set<std::string> unchecked;
    bool empty() const { return checked.empty() && unchecked.empty(); }
  };

  std::unordered_map<std::string, PerKey> data_;
};

}  // namespace gz_ros2_bridge_manager
