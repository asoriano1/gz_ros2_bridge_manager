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
- "Likely associated" shows topics that contain the model path or model name.
- "Bridgeable but unassigned" shows other bridgeable topics.
- "Unsupported types" shows topics with no known ROS 2 mapping (collapsed by default).

**Step 4 — Auto-check heuristic**

Select a model. Topics in the "Likely associated" section should be pre-checked.
Generic topics (`/clock`, `/scan`, `/tf`, …) should appear in "unassigned" unless
the topic path explicitly references the model (e.g. `/model/robot/scan`).

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

## Edge-case behaviors

| Scenario | Expected behavior |
|---|---|
| No Gazebo instance running | Status bar turns red: "No active Gazebo world found…" |
| World has zero models | Model drop-down is hidden; all topics go to unassigned |
| Topic type has no ROS 2 mapping | Topic appears in "Unsupported types", no bridge spec generated |
| Topic matches two different model names | Marked ambiguous (? indicator), moved to unassigned, not auto-checked |
| User checks a topic; it disappears on next refresh | Topic not included in command; yellow "1 previously selected topic not currently advertised" warning shown |
| Topic reappears after having been missing | Stored override re-checks it automatically |
| Two models both check the same global topic | Appears only once in the command when include-all-models is on |
| Manual mode (no model selected) | All bridgeable topics go to unassigned; user checks them manually; selections stored under `__manual__` key, isolated from per-model selections |
| Rapidly clicking Refresh | Only one discovery is in flight at a time (busy_ guard skips concurrent triggers) |
| Very long bridge command | Command display area is scrollable (Flickable); Copy still gets the full single-line command |
