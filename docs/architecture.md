# Architecture — gz_ros2_bridge_manager

## Why no URDF / SDF / robot_description dependency

The plugin operates entirely on a running Gazebo instance, not on robot source files.
This removes a whole class of fragility:

- No need to locate or parse the robot's source tree.
- Works regardless of how the robot was spawned (launch file, `gz service`, Fuel download).
- Works with robots that have no `robot_description` parameter published.
- Avoids the xacro-expansion + SDF-conversion pipeline and all its failure modes.

The trade-off is that topic→sensor association is heuristic rather than authoritative.
The heuristic is transparent (confidence labels per topic) and overridable per checkbox.

## Why gz-transport for discovery

Gazebo Harmonic (gz-sim 8) publishes a well-documented set of gz-transport topics
and services that fully describe the runtime state of a world:

- `/world/<name>/stats` — always present while a world is running; used to detect
  the world name without any configuration.
- `/world/<name>/scene/info` — service that returns the full scene graph including
  model names.
- `Node::TopicList()` + `Node::TopicInfo()` — enumerate all active publishers
  and resolve their message types.

These are stable public APIs. Using them requires no SDF/URDF parsing and no
reliance on internal implementation details.

## Why heuristic topic association

The `scene/info` service returns models and visuals but not the sensor→topic
mapping. Reconstructing that mapping without SDF source requires either:
- subscribing to `/world/<name>/state` and correlating sensor names to topics, or
- calling a dedicated sensor introspection service (not available in Harmonic as
  a stable API).

The heuristic approach (path-based pattern matching) handles the common case well
and degrades gracefully: topics that don't match are shown in a separate "unassigned"
list where the user can check them manually. The confidence label per topic makes
the reasoning transparent.

Association priority (highest wins):

| Category | Criterion |
|---|---|
| ExactModelPath | Topic contains `/model/<name>/` or `/world/<w>/model/<name>/` |
| ContainsSanitizedModelName | Sanitized model name (`rbvogui_xl` → `rbvogui_xl`) as a token |
| ContainsModelName | Raw model name as a token in the topic path |
| CompatibleButUnassigned | Bridgeable but no model match |
| Unsupported | No known ROS 2 type mapping |

Token matching uses non-identifier boundaries (`[^a-zA-Z0-9_]`), so `rbvogui`
does not match `/rbvogui_1/scan`. Generic topics (`/clock`, `/tf`, `/scan`, …)
are never auto-checked unless the path explicitly references the model.

## Why process launching is deferred

Running `QProcess` inside a Gazebo GUI plugin requires careful stdout/stderr
management to avoid flooding the Gazebo console, and lifecycle management
(kill on plugin close, restart on command change). That complexity is not needed
for the core "build the command" workflow. The planned implementation will use
`QProcess` with a dedicated status area.

## Code module split

```
include/gz_ros2_bridge_manager/
  BridgeTopicCandidate.hh       — data types shared across the pipeline
  BridgeTypeMapper.hh/.cpp      — static gz→ROS2 type table
  WorldDiscovery.hh/.cpp        — world name + model list
  GazeboTopicDiscovery.hh/.cpp  — topic enumeration + type resolution
  TopicAssociationHeuristic.hh/.cpp — classification engine
  ModelTopicSelectionStore.hh/.cpp  — per-model override state
  BridgeCommandBuilder.hh/.cpp  — spec list → shell command string
  BridgeSession.hh/.cpp         — full session: multi-model assembly + missing detection
  ModelSensorDiscovery.hh/.cpp  — placeholder (sensor hierarchy not yet implemented)
  Ros2BridgeManagerGui.hh/.cpp  — gz::gui::Plugin, Qt properties, background refresh
gui/
  Ros2BridgeManagerGui.qml      — presentational QML only; all logic in C++
```

**Design invariants:**

- `BridgeTypeMapper`, `TopicAssociationHeuristic`, `ModelTopicSelectionStore`,
  `BridgeCommandBuilder`, and `BridgeSessionBuilder` have zero Qt or Gazebo
  headers in their public interfaces. They are testable with plain gtest.
- `WorldDiscovery` and `GazeboTopicDiscovery` call gz-transport synchronously;
  the plugin always invokes them from a `QtConcurrent::run` worker.
- The QML file reads only Q_PROPERTYs and calls only Q_INVOKABLEs. No business
  logic lives in JavaScript bindings.

## Key data flow

```
[background thread]
  WorldDiscovery::discover()           → WorldInfo
  GazeboTopicDiscovery::discover()     → vector<GzTopicEntry>

[main thread, QueuedConnection]
  TopicAssociationHeuristic::associate()  → AssociationResult
  ModelTopicSelectionStore::applyOverrides()
  BridgeSessionBuilder::build()           → BridgeSessionResult
  emit topicsChanged(), bridgeCommandChanged()

[QML]
  reads Q_PROPERTYs, renders lists and command text
  calls Q_INVOKABLEs on user interaction
```
