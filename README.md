# gz_ros2_bridge_manager

Gazebo Harmonic GUI plugin for ROS 2 Jazzy that discovers ECM sensor topics,
maps them to `ros_gz_bridge`, and lets you run the resulting bridge directly
from the Gazebo UI.

## Gazebo ROS 2 Model Runtime Suite

This project is part of a broader **Gazebo ROS 2 Model Runtime Suite**: a set
of Gazebo GUI tools designed to take a robot or model from a description file
to a ROS 2-ready simulation runtime.

This package provides the **Gazebo ROS 2 Bridge Manager** step of that
workflow. It is intended to be used after the model is already present in the
Gazebo world, typically after importing it with
[`gz_model_importer_gui`](https://github.com/asoriano1/gz_model_importer_plugin).

```text
URDF / XACRO / SDF
       │
       ▼
gz_model_importer_gui
       │
       ├── Preview model
       ├── Spawn final model in Gazebo
       └── Optional robot_state_publisher for URDF/XACRO
       │
       ▼
Gazebo world + ROS 2 TF tree
       │
       ▼
gz_ros2_bridge_manager
       │
       ├── Discover ECM sensors and Gazebo topics
       ├── Build bridge specs
       └── Run ros_gz_bridge
       │
       ▼
ROS 2 tools and applications
RViz · Nav2 · MoveIt · custom nodes
```

## Current scope

Implemented in this plugin:

- ECM-first Gazebo sensor discovery
- Gazebo topic classification and bridge type mapping
- Compact model-by-model topic selection
- Direct `ros_gz_bridge` runtime control from the Gazebo UI
- Managed bridge process output and restart workflow

Provided by the companion importer:

- Model preview before final import
- Final URDF / XACRO / SDF spawn into Gazebo
- Optional `robot_state_publisher` launch for URDF / XACRO models

Not currently handled by this plugin:

- URDF / XACRO / SDF parsing
- `robot_description` generation
- `ros2_control` controller management
- Nav2 or MoveIt launch
- A full ROS 2 runtime process dashboard

## Demo

![Bridge manager demo](demo1.gif)

## What it does

- Discovers the active Gazebo world and the models currently present in it
- Reads ECM sensor entities and uses `components::SensorTopic` as the primary
  source of truth for topic assignment
- Matches advertised Gazebo topics to each sensor, including `camera_info`
  siblings when they are actually advertised
- Infers bridge types for common sensor topics when `TopicInfo` is not ready yet
  but the ECM sensor type is known
- Builds a deduplicated `parameter_bridge` command from the checked rows
- Runs, stops, and restarts the bridge process from the UI
- Captures bridge output and exposes compact debug details when needed

## What this enables

With the importer and bridge manager together, a typical user can:

1. Load a URDF, XACRO, or SDF model into a running Gazebo world.
2. Optionally start `robot_state_publisher` for URDF / XACRO models.
3. Discover the model's Gazebo sensor topics directly from the running world.
4. Launch the required `ros_gz_bridge` mappings from the Gazebo UI.
5. Connect RViz, Nav2, MoveIt, or custom ROS 2 nodes to the simulation runtime.

## Quick start

Build and source the package:

```bash
cd <workspace_root>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select gz_ros2_bridge_manager
source install/setup.bash
```

Start Gazebo with the packaged demo world and the plugin already loaded:

```bash
gz sim \
  $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/worlds/bridge_manager_demo.sdf \
  --gui-config $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/config/ros2_bridge_manager.config
```

Or load the plugin in your own world:

```bash
gz sim <your_world.sdf> \
  --gui-config $(ros2 pkg prefix gz_ros2_bridge_manager)/share/gz_ros2_bridge_manager/config/ros2_bridge_manager.config
```

The packaged config references the plugin as:

```xml
<plugin filename="Ros2BridgeManagerGui" name="ROS 2 Bridge Manager"/>
```

## Typical workflow

1. Load a robot into Gazebo, ideally with
   [`gz_model_importer_gui`](https://github.com/asoriano1/gz_model_importer_plugin)
2. Click `Refresh` or enable `Auto`
3. Expand the relevant model card under `Models:`
4. Review the compact sensor rows:
   `checkbox | topic | Gazebo → ROS 2 type`
5. Adjust the selected rows if needed
6. Use `Run` to start `ros_gz_bridge parameter_bridge`
7. Open `Bridge output` only when you need logs or diagnostics

## UI summary

- **Models accordion**: one card per Gazebo model
- **Compact sensor rows**: checked rows represent bridgeable topics already
  assigned to ECM sensors
- **Additional bridgeable topics**: only topics not claimed by any model sensor
- **Debug/details**: shows ECM `SensorTopic`, sensor type, type source,
  `TopicInfo` status, fallback path, and warnings
- **Bridge section**: status, `Run`, `Stop`, `Restart`, collapsed command view,
  and collapsed process output

## Matching strategy

The main model workflow is ECM-first:

- `SensorTopic` from the ECM is the authoritative link between a sensor entity
  and its Gazebo topic
- If `TopicInfo` exposes the message type, that advertised type is used
- If `TopicInfo` is still unavailable, common sensor types are inferred from the
  ECM sensor component:
  - `camera` -> `gz.msgs.Image` -> `sensor_msgs/msg/Image`
  - `imu` -> `gz.msgs.IMU` -> `sensor_msgs/msg/Imu`
  - `lidar` / `gpu_lidar` / `ray` -> `gz.msgs.LaserScan` -> `sensor_msgs/msg/LaserScan`
  - `navsat` / `gps` -> `gz.msgs.NavSat` -> `sensor_msgs/msg/NavSatFix`
- `camera_info` is only attached when it is actually advertised in Gazebo
- Topics already claimed under a model are excluded from `Additional bridgeable topics`

## Bridge runtime control

The plugin can launch the bridge directly with:

```bash
ros2 run ros_gz_bridge parameter_bridge <specs...>
```

Runtime behavior:

- `Run` starts exactly the currently selected bridge specs
- `Stop` terminates the managed bridge process group
- `Restart` is enabled when the selection changes while the bridge is running
- `Bridge output` captures stdout/stderr from the managed bridge process

## Example command

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /sensor_test_robot_urdf_1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /sensor_test_robot_urdf_1/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /sensor_test_robot_urdf_1/imu/data_raw@sensor_msgs/msg/Imu@gz.msgs.IMU
```

## Test

```bash
cd <workspace_root>
colcon test --packages-select gz_ros2_bridge_manager
colcon test-result --verbose
```

The test suite covers:

- Gazebo type to ROS 2 type mappings
- ECM sensor extraction and `SensorTopic` handling
- Topic matching and deduplication
- Bridge command generation
- Process manager state transitions
- Per-model selection state

## Architecture

```text
Ros2BridgeManagerGui (gz::sim::GuiSystem)
│
├── WorldDiscovery
├── GazeboTopicDiscovery
├── EcmSensorExtractor
├── EcmTopicMatcher
├── BridgeTypeMapper
├── ModelTopicSelectionStore
├── BridgeCommandBuilder
└── BridgeProcessManager
```

The UI layer stays thin: discovery and matching live in pure or near-pure C++
helpers, while QML renders the accordion and bridge controls.

See [`docs/architecture.md`](docs/architecture.md) for more detail.

## Current limitations

- Session selections are not persisted across plugin restarts
- Only common sensor topic types are inferred when `TopicInfo` is missing
- Non-standard plugin topics still depend on what Gazebo advertises at runtime
- The plugin manages only the bridge process it starts itself

## Author

Ángel Soriano — [Robotnik Automation S.L.L.](https://robotnik.es)
