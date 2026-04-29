# Manual Testing Checklist — gz_ros2_bridge_manager

## Prerequisites

- ROS 2 Jazzy sourced.
- Gazebo Harmonic installed.
- Workspace built and sourced (`source install/setup.bash`).
- A Gazebo world with at least one robot model running.

---

## Golden-path checklist

**Step 1 — Plugin loads**

Launch Gazebo and open the plugin from the Plugins menu (or use the config file).
Expected: the panel shows "Not yet refreshed" in the status area and an enabled
Refresh button. No errors in the Gazebo console.

**Step 2 — Refresh discovers the world**

Click Refresh.
Expected: status bar turns green and shows `World: <name>  •  Models: N  •  Topics: M`.
The model drop-down is populated with the discovered model names.

**Step 3 — Topic lists populate**

After refresh with a model selected:
- "Detected sensors in selected model" (green card) shows ECM-confirmed sensors.
- "Heuristic suggestions" shows additional topic-name matches (collapsed when ECM active).
- "Additional bridgeable topics" shows other bridgeable topics not linked to sensors.
- "Unsupported / debug topics" shows topics with no known ROS 2 mapping (collapsed).

**Step 4 — Auto-check heuristic**

Select a model. Topics in the "Heuristic suggestions" section should be pre-checked.
Generic topics (`/clock`, `/scan`, `/tf`, …) should appear in "additional bridgeable topics"
unless the topic path explicitly references the model (e.g. `/model/robot/scan`).

**Step 5 — Bridge command appears**

With at least one topic checked, the "Bridge command" card turns green and shows
a `ros2 run ros_gz_bridge parameter_bridge ...` command. The selection summary
updates to "N topic(s) selected".

**Step 6 — Copy to clipboard**

Click Copy. Paste into a terminal with `Ctrl+Shift+V`. Verify the full command
including all specs is present.

**Step 7 — Manual checkbox toggle**

Uncheck a pre-checked topic. The command updates immediately and loses that spec.
Re-check it; the command gains the spec back. Switch to a different model and back;
the override is remembered (the manually unchecked topic stays unchecked).

**Step 8 — Reset restores heuristic defaults**

After manually unchecking a topic, click Reset. The heuristic defaults are restored
and the previously unchecked topic is auto-checked again.

**Step 9 — Include-all-models mode**

Curate two different models (check topics for each). Enable "Include checked topics
from all models". The command should contain specs from both models, deduplicated.
The selection summary shows "N from current  +  M from other models".

**Step 10 — Auto-refresh**

Enable the "Auto" checkbox. Wait ~5 seconds. Verify the "↻ HH:MM:SS" timestamp
updates automatically without clicking Refresh. Verify the Busy indicator appears
and disappears each cycle.

**Step 11 — Model gone**

While a model is selected and the plugin is auto-refreshing, remove the model from
the world (e.g. `gz service -s /world/default/remove --reqtype gz.msgs.Entity ...`).
On the next refresh, the orange "model gone" warning banner should appear and the
drop-down should fall back to "(no model — manual selection)". Previously curated
selections are preserved.

**Step 12 — Paste command into terminal and run**

Copy the generated command, open a sourced ROS 2 terminal, and run it. Verify:
- `ros2 topic list` shows the expected bridged topics.
- `ros2 topic echo <topic>` receives data.

---

## ECM sensor discovery checklist

These steps validate that the ECM-based sensor hierarchy discovery is working
correctly. Run them with a world that has a robot model with at least one sensor
(lidar, camera, or IMU).

**ECM-1 — ECM status active**

After loading the plugin with Gazebo running:
- The model selector card shows a green dot next to "Sensor discovery: ECM active".
- The `sensorDiscoveryStatus` field shows "N sensors across M model(s)".

**ECM-2 — Select model and see sensor count**

Select a model from the dropdown.
Expected:
- The green "Detected sensors in selected model (N)" card appears.
- The count matches the number of sensors defined in the model's SDF/URDF.
- Each sensor row shows: `link_name / sensor_name` | type | source label.

**ECM-3 — Match source label is correct**

For each sensor row, verify the "source:" label:
- If the sensor has a `<topic>` element in its SDF: label shows "SensorTopic exact"
  or "SensorTopic prefix".
- If the sensor uses the default Gazebo path `/world/<w>/model/<m>/…`: label shows
  "Gazebo standard prefix".
- Unresolved (topic not advertised): shows "Unresolved" in red.

**ECM-4 — Topic prefix shown**

For each sensor:
- If `SensorTopic` is populated: the "topic: /custom/path" line is shown.
- If only the fallback path is available: the "path: /world/…/sensor/…" line is shown.

**ECM-5 — Per-topic checkboxes**

Verify that within each sensor card, each matched topic has a checkbox.
- ECM-confirmed topics are checked by default.
- Toggling a checkbox in the sensor card updates the bridge command immediately.

**ECM-6 — Topics appear in `gz topic -l`**

Run `gz topic -l` in a terminal. Verify that the topics shown in the "Detected sensors"
card match advertised Gazebo topics.

**ECM-7 — Generated command contains matched sensor topics**

With a model selected and ECM data available, verify the bridge command contains
the specs for matched sensor topics. Example for a lidar:
```
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/my_robot/link/laser_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**ECM-8 — Pause simulation and refresh**

Pause Gazebo. Click Refresh.
Expected:
- Topics may disappear from the advertised list (Gazebo may stop publishing).
- The sensor card still shows the sensor (ECM data persists).
- Unresolved sensors show: "⚠ topic not currently advertised; resume simulation or refresh."
- The bridge command loses unchecked/unresolved topics.

**ECM-9 — Resume simulation and refresh**

Resume Gazebo. Click Refresh.
Expected:
- Topics reappear.
- Previously unresolved sensors resolve automatically (green border, matched topics shown).
- Checkboxes reflect stored overrides.

**ECM-10 — SensorTopic unavailable (fallback path used)**

If the robot SDF has sensors without explicit `<topic>` elements:
- The "path: /world/…/sensor/…" line is shown for those sensors.
- "source: Gazebo standard prefix" appears.
- Matching still works if the sensor is publishing on the standard path.

**ECM-11 — Nested model detection**

If the model contains nested models (model-within-a-model):
- Sensors inside nested models show a "nested" badge in the sensor row.
- Matching still works normally.

**ECM-12 — ECM unavailable banner**

If Gazebo is running but the ECM is not yet replicated (e.g. world just started):
- Model selector shows a grey dot: "Sensor discovery: ECM unavailable".
- A yellow "⚠ ECM sensor discovery unavailable. Falling back to topic-name heuristics."
  banner appears when a model is selected.
- Heuristic suggestions are expanded and shown as the primary discovery method.

---

## Edge-case behaviors

| Scenario | Expected behavior |
|---|---|
| No Gazebo instance running | Status bar turns red: "No active Gazebo world found…" |
| World has zero models | Model drop-down is hidden; all topics go to additional bridgeable topics |
| Topic type has no ROS 2 mapping | Topic appears in "Unsupported / debug topics", no bridge spec generated |
| Topic matches two different model names | Marked ambiguous (? indicator), moved to additional, not auto-checked |
| User checks a topic; it disappears on next refresh | Topic not included in command; yellow "1 previously selected topic not currently advertised" warning shown |
| Topic reappears after having been missing | Stored override re-checks it automatically |
| Two models both check the same global topic | Appears only once in the command when include-all-models is on |
| Manual mode (no model selected) | All bridgeable topics go to additional; user checks them manually; selections stored under `__manual__` key, isolated from per-model selections |
| Rapidly clicking Refresh | Only one discovery is in flight at a time (busy_ guard skips concurrent triggers) |
| Very long bridge command | Command display area is scrollable (Flickable); Copy still gets the full single-line command |
| Sensor with SensorTopic but no advertised topic | Sensor row shows "Unresolved" badge; not included in bridge command |
| ECM active but heuristic section empty | "Heuristic suggestions" section hidden; only ECM card and additional topics visible |
