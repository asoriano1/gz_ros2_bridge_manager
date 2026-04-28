#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gz_ros2_bridge_manager/BridgeTopicCandidate.hh"

namespace gz_ros2_bridge_manager
{

// Per-model user-curated topic selection store.
//
// Keys are stable strings of the form:
//   "<world>::<model>"      for a real model
//   "<world>::__manual__"   for the no-model / manual selection mode
//
// Each key carries two disjoint sets of explicit overrides:
//   - manuallyChecked   — topic was explicitly checked by the user
//   - manuallyUnchecked — topic was explicitly unchecked by the user
//
// Topics absent from both sets keep the heuristic's default `checked` state.
//
// Pure C++ — no Qt, no Gazebo. Trivially testable.
class ModelTopicSelectionStore
{
public:
  static constexpr const char *kManualMarker = "__manual__";
  static constexpr const char *kKeySeparator = "::";

  // ---- Key construction & decomposition ----------------------------------
  static std::string keyForModel(const std::string &world,
                                 const std::string &model);
  static std::string manualKey(const std::string &world);
  static bool        isManualKey(const std::string &key);
  // Returns the world part of the key, or "" if no separator is found.
  static std::string worldOfKey(const std::string &key);
  // Returns the model part of the key, or "" for manual keys.
  static std::string modelOfKey(const std::string &key);

  // ---- Override management -----------------------------------------------
  // Records an explicit user choice for `topic` under `key`.  The two sets
  // are kept disjoint: setting checked=true removes from manuallyUnchecked
  // and vice-versa.
  void setOverride(const std::string &key, const std::string &topic,
                   bool checked);

  // Removes `topic` from both sets (returns to heuristic default).
  void clearOverride(const std::string &key, const std::string &topic);

  // Drops every override for the key (returns whole key to defaults).
  void resetKey(const std::string &key);

  // Removes the key entirely.
  void forgetKey(const std::string &key);

  // ---- Application --------------------------------------------------------
  // For each candidate where the topic appears in this key's overrides,
  // updates `checked` accordingly. Non-bridgeable candidates are untouched.
  void applyOverrides(const std::string &key,
                      std::vector<BridgeTopicCandidate> &candidates) const;

  // ---- Inspection ---------------------------------------------------------
  bool hasKey(const std::string &key) const;
  bool isManuallyChecked  (const std::string &key, const std::string &topic) const;
  bool isManuallyUnchecked(const std::string &key, const std::string &topic) const;
  bool hasAnyOverride     (const std::string &key, const std::string &topic) const;

  std::vector<std::string>          allKeys() const;
  std::unordered_set<std::string>   manuallyChecked  (const std::string &key) const;
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
