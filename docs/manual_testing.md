# Manual Testing Checklist — gz_ros2_bridge_manager

## Prerequisites

- ROS 2 Jazzy sourced.
- Gazebo Harmonic installed.
- Workspace built and sourced (`source install/setup.bash`).
- A Gazebo world with at least one robot model running.

---

## Golden-path checklist (compact accordion workflow)

**Step 1 — Start Gazebo**

Start Gazebo with a world that contains at least one robot model with sensors.

**Step 2 — Load the plugin and refresh**

Open the plugin from the Plugins menu (or use the config file), then click Refresh.
Expected:
- The panel starts with "Not yet refreshed" and an enabled Refresh button.
- After Refresh, the status bar turns green and shows `World: <name>  •  Models: N  •  Topics: M`.
- One accordion card appears per discovered model.

**Step 3 — Expand a model card**

Click a model header / arrow.
Expected:
- The card expands without affecting the other model cards.
- The header shows the model name, ECM status dot, selected topic count, and Reset button.

**Step 4 — Toggle several topic checkboxes**

Inside the expanded card, check and uncheck multiple sensor-topic rows.

**Step 5 — Confirm the card stays expanded**

Expected:
- The same model card remains expanded after every checkbox toggle.
- Only clicking the model header / arrow changes the expanded state.

**Step 6 — Confirm the bridge command count updates**

Expected:
- The `N selected` count in the model header updates immediately.
- The Bridge command card count updates immediately.
- The generated command text changes immediately when topics are checked or unchecked.

**Step 7 — Confirm the normal UI is compact**

With Debug/details disabled:
- Each matched sensor topic appears as one compact row:
  `[x] sensor_name   /gazebo/topic   GazeboType → ros2/type`
- Sensors with multiple matched topics show multiple compact rows.
- Sensors with no matched topic show a single compact warning row:
  `⚠ sensor_name   no advertised topic found`
- Source labels, declared topic lines, fallback paths, and detailed warning text are hidden.

**Step 8 — Enable Debug/details**

Turn on the `Debug/details` toggle in the header.

**Step 9 — Confirm debug details appear only in debug mode**

Expected:
- Source / match source lines appear.
- Declared topic lines appear when available.
- Fallback path lines appear when no declared topic exists.
- Detailed warning / weak-match text appears only in debug mode.

**Step 10 — Disable Debug/details again**

Turn `Debug/details` off.
Expected: all source / fallback / detailed warning text disappears and the compact rows remain.

---

## ECM sensor discovery checklist

These steps validate that the ECM-based sensor hierarchy discovery is working
correctly. Run them with a world that has a robot model with at least one sensor
(lidar, camera, or IMU).

**ECM-1 — ECM dot indicator**

After Refresh:
- A model with ECM sensor data shows a green dot in the card header.
- A model with no ECM sensor data shows a grey dot.

**ECM-2 — Compact matched-topic rows**

Expand the model card.
Expected:
- Each matched sensor topic appears as a compact row.
- ECM-confirmed topics are checked by default.
- Weak matches can show a subtle `verify` marker in normal mode.

**ECM-3 — Unresolved sensors**

For a sensor whose topic is not currently advertised:
- No matched topic row is shown.
- A compact warning row appears: `⚠ sensor_name   no advertised topic found`.

**ECM-4 — Debug/details exposes internal match data**

Enable `Debug/details`.
Expected:
- `source: ECM exact`, `ECM prefix`, `ECM path`, `Name match`, `Type fallback`, or `Unresolved` appears per sensor.
- `declared: /topic/...` appears when the sensor had an explicit topic.
- `path: /world/.../sensor/...` appears when the fallback prefix is being used.
- Warning text appears only in debug mode.

**ECM-5 — Topics appear in `gz topic -l`**

Run `gz topic -l` in a terminal.
Verify the topics shown in the compact sensor rows match advertised Gazebo topics.

**ECM-6 — Generated command contains matched sensor topics**

With a model expanded and ECM data available, verify the bridge command contains
the specs for matched sensor topics. Example for a lidar:
```
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/my_robot/link/laser_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

**ECM-7 — Pause simulation and refresh**

Pause Gazebo. Click Refresh.
Expected:
- Topics may disappear from the advertised list (Gazebo may stop publishing).
- Sensor rows still show the sensor name (ECM data persists).
- Unresolved sensors show the compact warning row.

**ECM-8 — Resume simulation and refresh**

Resume Gazebo. Click Refresh.
Expected:
- Topics reappear.
- Previously unresolved sensors resolve automatically.
- Checkbox overrides are preserved and expanded cards stay expanded.

**ECM-9 — SensorTopic unavailable (fallback path used)**

For sensors without explicit `<topic>` elements in SDF:
- `path: /world/…/sensor/…` appears in Debug/details.
- `source: ECM path` appears in Debug/details.
- Matching still works if the sensor is publishing on the standard path.

---

## Edge-case behaviors

| Scenario | Expected behavior |
|---|---|
| No Gazebo instance running | Status bar turns red: "No active Gazebo world found…" |
| World has zero models | No model cards; all topics go to additional bridgeable topics |
| Topic type has no ROS 2 mapping | Topic appears in "Unsupported / debug topics", no bridge spec generated |
| User checks a topic; it disappears on next refresh | Topic is removed from the generated command until it is advertised again |
| Topic reappears after having been missing | Stored override re-checks it automatically |
| Two models both check the same global topic | Appears only once in the command (BridgeCommandBuilder deduplicates by spec) |
| Additional topic checked | Appears in bridge command after all per-model topics |
| Rapidly clicking Refresh | Only one discovery is in flight at a time (busy_ guard skips concurrent triggers) |
| Very long bridge command | Command display area is scrollable (Flickable); Copy still gets the full single-line command |
| Sensor with SensorTopic but no advertised topic | Normal mode shows the compact warning row; Debug/details shows declared topic / fallback path and warning text |
| Model removed between refreshes | Card disappears on next refresh; stored selections preserved; re-adding restores them |
| Toggling a topic checkbox | The model card stays expanded; only clicking the header / arrow collapses it |
