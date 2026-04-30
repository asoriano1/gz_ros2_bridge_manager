# gz_ros2_bridge_manager

A Gazebo Harmonic GUI plugin that discovers active Gazebo Transport topics and
helps you build a `ros2 run ros_gz_bridge parameter_bridge` command — without
reading URDF, SDF, `robot_description`, or any metadata file.

---

## Capabilities

- **World and model discovery** — detects the active Gazebo world name and the
  models in it using gz-transport (`/world/<name>/stats` topic + `/world/<name>/scene/info` service).
- **Topic discovery** — enumerates all gz-transport topics, resolves each
  publisher's message type, and maps it to the corresponding ROS 2 type.
- **Heuristic topic association** — classifies topics per model with four
  confidence levels (exact model path → sanitized name → raw name → unassigned).
  Generic topics (`/clock`, `/scan`, `/tf`, …) are never auto-associated unless
  the topic path explicitly references the model.
- **Per-model selection memory** — manual checkbox overrides persist for the
  lifetime of the session, per model. Switching models and back restores your
  choices.
- **Include-all-models mode** — unions the checked topics from every curated
  model in the world into a single bridge command, with deduplication.
- **Missing-topic warning** — topics you explicitly checked in a previous
  discovery round that are no longer advertised are flagged rather than silently
  dropped.
- **Model-gone detection** — if the selected model disappears from the world on
  refresh, the plugin falls back to manual mode and shows a warning.
- **Auto-refresh** — optional 2.5 s polling timer; skips a tick if a refresh is
  already in flight.
- **Copy to clipboard** — single-click copy of the complete command.

---

## Non-goals and limitations

- **No process management.** The plugin generates the command; you run it in a
  terminal. QProcess-based launch is a planned future feature.
- **No session persistence.** Checkbox overrides reset when the plugin is closed.
  JSON profile save/load is a planned future feature.
- **No URDF/SDF/xacro parsing.** Discovery is entirely via gz-transport at runtime.
- **No sensor hierarchy.** The `scene/info` service exposes models and visuals
  but not the sensor→topic mapping. Heuristic topic matching compensates for
  this gap.
- **No namespace/frame-prefix rewriting.** Selecting a model namespace and
  `--ros-args` remapping is left to the user. The plugin cannot rewrite
  hardcoded plugin topics inside arbitrary robot descriptions.
- **No Fuel integration, thumbnails, or drag-and-drop.** This is a bridge
  command helper, not a robot catalog.

---

## Build

Requires ROS 2 Jazzy, Gazebo Harmonic, and a colcon workspace.

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select gz_ros2_bridge_manager
source install/setup.bash
```

---

## Test

```bash
cd <workspace_root>
colcon test --packages-select gz_ros2_bridge_manager
colcon test-result --verbose
```

Five test binaries run (~53 tests total, no Gazebo instance required):

| Binary | What it covers |
|---|---|
| `test_bridge_type_mapper` | Type mappings and bridge spec format |
| `test_topic_association_heuristic` | Classification, sanitization, ambiguity |
| `test_bridge_command_builder` | Command generation and deduplication |
| `test_model_topic_selection_store` | Override management, key helpers |
| `test_bridge_session` | Full session build pipeline |

---

## Loading the plugin

After building and sourcing the workspace, the install step registers the plugin
path via an ament environment hook. Launch Gazebo and open the plugin from the
Plugins menu:

```bash
gz sim <your_world.sdf>
```

For a direct quick start with the plugin already loaded, the package now installs
both a GUI config and a small demo world with a camera sensor:

```bash
gz sim \
  $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/worlds/bridge_manager_demo.sdf \
  --gui-config $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/config/ros2_bridge_manager.config
```

If you want to use your own world instead, load the plugin via the packaged
config file:

```bash
gz sim <your_world.sdf> \
  --gui-config $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/config/ros2_bridge_manager.config
```

The config file references the plugin as:

```xml
<plugin filename="Ros2BridgeManagerGui" name="ROS 2 Bridge Manager"/>
```

---

## Usage

1. **Quick start** (optional) — launch the packaged `bridge_manager_demo.sdf`
   world if you want the bridge manager to find a sensor-bearing model
   immediately after startup.
2. **Refresh** — click Refresh (or enable Auto) to discover the active Gazebo world,
   its models, and all advertised topics.
3. **Select a model** — pick a model from the drop-down. Topics likely associated
   with that model appear pre-checked in the "Likely associated" list.
4. **Tune the selection** — check or uncheck individual topics. Use "Check all" or
   "Uncheck all" as starting points. "Reset" restores heuristic defaults.
5. **Include all models** (optional) — enable the checkbox to union the checked
   topics from every model you have curated in the world into one command.
6. **Copy** — click Copy to copy the command to the clipboard, then paste it into
   a sourced ROS 2 terminal.

---

## Example command output

```
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/rbvogui_xl/link/laser_front_link/sensor/laser_front/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /world/default/model/rbvogui_xl/link/camera_front_link/sensor/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```

---

## Architecture summary

```
Ros2BridgeManagerGui (gz::gui::Plugin)
│
├── WorldDiscovery          — world name + model list (gz-transport)
├── GazeboTopicDiscovery    — topic enumeration + message types
├── BridgeTypeMapper        — gz type → ROS 2 type mappings
│
├── TopicAssociationHeuristic  — classifies topics per model
├── ModelTopicSelectionStore   — per-model checkbox override state
│
├── BridgeSessionBuilder    — cross-model command assembly
└── BridgeCommandBuilder    — spec list → command string
```

The GUI layer (`Ros2BridgeManagerGui` + QML) owns no business logic. Discovery
runs on a `QtConcurrent` background thread; results are delivered to the main
thread via `QMetaObject::invokeMethod(Qt::QueuedConnection)`.

See [`docs/architecture.md`](docs/architecture.md) for design rationale.

---

## Roadmap

| Feature | Status |
|---|---|
| Topic discovery + heuristic association | Done |
| Per-model selection memory | Done |
| Include-all-models mode | Done |
| Auto-refresh | Done |
| JSON profile persistence | Planned |
| Bridge process launch (QProcess) | Planned |
| `/world/<w>/stats` subscription (event-driven refresh) | Planned |
| Model/sensor hierarchy from SDF introspection | Planned |
