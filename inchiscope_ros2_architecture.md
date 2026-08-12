# Inchiscope ROS2 Architecture

Target platform: **ROS2 Jazzy**, Ubuntu 24.04, Python-first (`rclpy`) for all logic nodes.
Firmware: **PlatformIO** on Arduino Mega, C++, servo-loop only (no kinematics, no calibration math).

This document is the original handoff spec for implementation in Claude Code. It defines the
workspace layout, the serial protocol between PC and Mega, the custom ROS2 interfaces, the node
responsibilities, and the build/test phasing agreed in planning. Sections 2 and 5 have been
updated to match what's actually implemented where reality diverged from the original plan
(mainly: `inchiscope_aurora` ended up C++/`ament_cmake`, not Python, and both it and
`inchiscope_camera` are further along and shaped differently than originally sketched). See the
top-level `README.md` for current status, the command reference, and per-package `README.md`s for
implementation detail — this document stays the record of the *decisions*, not a live status page.

---

## 1. Design decisions locked in

- All kinematics (constant-curvature model, cubic piston↔bellows inversion) and all calibration
  (AB pressure↔diameter) run **on the PC**, in Python. Firmware only executes length/pressure
  setpoints and reports raw sensor readings.
- Firmware telemetry rate: **20 Hz** (matches realistic I2C mux + MPRLS read latency across 3
  channels — verify on bench once firmware is simplified, adjust if needed).
- AB anchoring balloons are named **`proximal`**, **`central`**, **`distal`** (matches the
  paper's P-AB/C-AB/D-AB labelling) rather than numeric indices.
- AB diameter control is **open-loop feedforward** via a placeholder pressure↔diameter curve,
  with a **hard pressure ceiling per AB** derived from the anchoring-force characterisation
  (paper §III-A) as a safety backstop. This will become closed-loop once the soft diameter
  sensor is integrated — the `ab_control_node` external interface (diameter in) will not need
  to change when that happens.
- Aurora EM tracking: **one 6DOF sensor now**, mounted at the distal tip alongside the camera.
  Interfaces are namespaced by sensor index (`sensor_0`) from day one so that adding two more
  sensors (one per AB, in future) is a config change, not a refactor.
- High-level mode transitions (anchoring, full-extension inching, front-steering, anchored
  observation) use **ROS2 actions**, not services — they have real settling time (AB inflation,
  contact confirmation) and benefit from feedback + cancellation + a definite result. Continuous
  streams (pressure, piston position, pose) stay as plain topics.

---

## 2. Workspace layout

```
inchiscope_ws/
├── firmware/
│   └── inchiscope_mega/            # PlatformIO project (C++, not part of colcon build)
│       ├── platformio.ini
│       └── src/main.cpp
└── src/
    ├── inchiscope_msgs/            # ament_cmake, rosidl interfaces only (msg + action)
    ├── inchiscope_serial_bridge/   # ament_python — talks to Mega, publishes telemetry
    ├── inchiscope_pba_control/     # ament_python — CC model, piston length targets
    ├── inchiscope_ab_control/      # ament_python — diameter → pressure, safety clamp
    ├── inchiscope_control/         # ament_python — state machine, action servers
    ├── inchiscope_aurora/          # ament_cmake, C++ — EM tracker pose (see note below)
    ├── inchiscope_camera/          # ament_python — NanEye feed via capture card
    └── inchiscope_bringup/         # launch files, params, rosbag2 recording config
```

Interface generation (`.msg`/`.action`) requires `rosidl`, which needs a small `ament_cmake`
package (`inchiscope_msgs`) even though most of the workspace is Python.

**Resolved from the original open item below:** neither SDK needed a pybind11/ctypes wrapper, but
for opposite reasons, and `inchiscope_aurora` ended up C++ rather than Python:

- NDI's Aurora SDK ("Combined API Sample C++") is a C++ source-only sample with no Python
  bindings and no prebuilt shared library to wrap — the practical path was to make
  `inchiscope_aurora` a real `ament_cmake` C++ package and compile the vendor `.cpp` sources
  directly into `aurora_tracker_node`, linking `rclcpp`/`tf2`/`tf2_ros` instead of `rclpy`. It's
  the one node in the workspace that isn't Python. The vendor SDK itself is **not vendored into
  this repo** (proprietary NDI sample code, no redistribution grant in its `license.txt`) — see
  `src/inchiscope_aurora/README.md`.
- The NanEye camera turned out not to need a vendor SDK at all: it's fed through a USB capture
  card that the kernel already exposes as a plain V4L2/UVC video device, so `inchiscope_camera`
  is ordinary `ament_python` using OpenCV's `cv2.VideoCapture` against the device node — no driver
  wrapper needed.

~~**Open item:** the Aurora and NanEye SDKs are typically C/C++. If no Python bindings exist,
`inchiscope_aurora` / `inchiscope_camera` may need a thin pybind11 or ctypes wrapper around a
vendor `.so`/driver rather than pure Python. Confirm SDK language before implementation — flag
to Claude Code as a spike/investigation task, not an assumption.~~

---

## 3. Serial protocol (PC ↔ Mega)

Replaces the current single-character command scheme entirely. ASCII, newline-terminated,
human-readable for bench debugging. Firmware does no parsing beyond splitting on whitespace and
converting numbers — no math.

**PC → Mega (commands):**
```
PISTON <id> <target_mm>        # id ∈ {p1,p2,p3,d1,d2,d3}
VALVE <ab_id> <duty_0_100>      # ab_id ∈ {proximal,central,distal}
PING
```

**Mega → PC (telemetry, pushed at 20 Hz, not polled):**
```
TEL <t_ms> <p_prox_kpa> <p_cent_kpa> <p_dist_kpa> <p1_mm> <p2_mm> <p3_mm> <d1_mm> <d2_mm> <d3_mm>
ACK <command_echo>
ERR <message>
```

Firmware responsibilities only: read 3 pressure sensors via mux, PWM the 3 valves toward
commanded duty cycle, step the 3 linear actuators toward commanded lengths (reusing the existing
stepper step tables), and push `TEL` lines on a fixed timer. All CC model code, cubic inversion
coefficients (`a,b,c,d` in current `main.cpp`), and joystick handling should be **deleted** from
firmware, not ported.

---

## 4. Custom interfaces (`inchiscope_msgs`)

### Messages

```
# msg/PistonState.msg
string id
float32 length_mm
float32 target_length_mm
bool at_target

# msg/PressureState.msg
string ab_id
float32 pressure_kpa
float32 target_pressure_kpa
bool at_target

# msg/InchiscopeState.msg
string mode              # idle | full_extension | front_steering | anchored_observation | calibrating
string phase             # free text, human-readable sub-state for the UI
bool anchored
```

### Actions

```
# action/SetAbDiameter.action
string ab_id
float32 target_diameter_mm
---
bool success
float32 final_pressure_kpa
string message
---
float32 current_pressure_kpa
float32 elapsed_sec

# action/MovePba.action
string pba_id            # proximal | distal
float32 arc_length_mm
float32 bend_angle_rad
float32 bend_plane_rad
---
bool success
float32[3] final_piston_lengths_mm
string message
---
float32[3] current_piston_lengths_mm

# action/TransitionMode.action
string target_mode
---
bool success
string message
---
string current_phase
```

---

## 5. Nodes

### `serial_bridge_node` (inchiscope_serial_bridge)
- Owns the serial port. Translates `PistonState[]`/`PressureState[]` telemetry lines into
  publishes at 20 Hz. Subscribes to low-level command topics and forwards them as protocol lines.
- Topics out: `/firmware/piston_state` (PistonState[]), `/firmware/pressure_state` (PressureState[])
- Topics in: `/firmware/piston_cmd` (per-actuator target), `/firmware/valve_cmd` (per-AB target)
- No kinematics, no calibration — pure translation layer.

### `pba_control_node` (inchiscope_pba_control)
- Hosts the `MovePba` action server for both `proximal` and `distal` PBAs.
- Runs the constant-curvature model + cubic piston↔bellows inversion (ported from current
  `main.cpp`, unchanged math) to convert `(L, θ, φ)` → 3 bellows lengths → 3 piston targets.
- Publishes piston targets to `/firmware/piston_cmd`, watches `/firmware/piston_state` for
  `at_target` to drive action feedback/result.
- Also exposes a lighter joystick-style topic input for teleoperation of the distal PBA, bypassing
  the action interface when direct teleop is wanted.

### `ab_control_node` (inchiscope_ab_control)
- Hosts the `SetAbDiameter` action server for `proximal`/`central`/`distal`.
- Maps target diameter → pressure via a **placeholder** curve (swap-in point once you derive the
  real fit from Fig. 4a-I data).
- Applies a **hard pressure ceiling per AB**, configurable via params, derived from the
  anchoring-force data (paper §III-A) — reject/clamp any request above it and report in the
  action result rather than silently exceeding it.
- Publishes to `/firmware/valve_cmd`, watches `/firmware/pressure_state`.

### `inchiscope_control_node` (inchiscope_control)
- State machine hosting the `TransitionMode` action server. States: `idle`, `calibrating`,
  `full_extension`, `front_steering`, `anchored_observation`.
- On a mode transition, sequences calls to the `SetAbDiameter` and `MovePba` action clients (e.g.
  the inching cycle in Fig. 3b) and publishes `/inchiscope/state` continuously so any subscriber
  (operator UI, logger) can observe progress without blocking on the action.

### `aurora_tracker_node` (inchiscope_aurora) — **implemented, hardware-verified**
- C++, built directly against NDI's Combined API sample sources (see workspace-layout note
  above). Connects to the field generator over serial, then auto-detects **both** tools with a
  plain port search (`PHSR`) rather than requesting a specific handle — Aurora has no `PHRQ`
  command, unlike Polaris/Vega. Tells the reference tool (has chip data, `PINIT` succeeds
  immediately) apart from the bare 6D sensor (no chip, `PINIT` fails until its virtual SROM file
  is uploaded via `PVWR`) without needing to know which physical port either is wired into.
  Retries the whole connect/init/enable/start-tracking sequence on a timer so it's safe to launch
  before the Aurora unit is powered on.
- Publishes three `PoseStamped` topics (all in `geometry_msgs/msg/PoseStamped`), plus matching
  `tf2` transforms for the two raw poses:
  - `/aurora/reference/pose`, `/aurora/sensor_0/pose` — raw poses in the field generator's own
    frame (`aurora_field`).
  - `/aurora/sensor_0/pose_relative_to_reference` — the sensor's pose composed into the
    reference tool's frame. Since the reference sits flat on the table, this is effectively the
    sensor's pose in a bench-fixed world frame, which cancels out the field generator's arbitrary
    internal frame — this is the topic to record for offline reconstruction, not the raw
    `sensor_0/pose`. Only published when both tools are in view this cycle.
  - Namespaced (`sensor_0`) for a future `sensor_1`, `sensor_2` as originally planned.
- Full setup (vendor SDK placement, `.rom` file, params) in `src/inchiscope_aurora/README.md`.

### `camera_node` (inchiscope_camera) — **implemented**
- Publishes the NanEye feed on `/camera/image_raw` via `cv2.VideoCapture` against the capture
  card's V4L2 device node. Defaults to `pixel_format: MJPG`, `1920x1080` — the capture card's
  YUYV fallback (what you get with no explicit FOURCC) both defaults to a 4:3 800x600 mode and
  caps out well below 30fps at 16:9 sizes, confirmed via `v4l2-ctl --list-formats-ext`; re-check
  that command on different capture-card hardware. `/camera/camera_info` still isn't published —
  add it once the lens/sensor has been calibrated, as originally planned.
- `camera_viewer_node` — added beyond the original spec: a separate node subscribing to
  `/camera/image_raw` and showing it in a `cv2.imshow` window, kept out of `camera_node` itself
  so the capture/publish path can run headless. See `src/inchiscope_camera/README.md`.

### `inchiscope_bringup`
- Launch files: `phase1_bridge.launch.py`, `aurora.launch.py` (+ optional RViz, `use_rviz` arg),
  `camera.launch.py` (+ optional viewer, `show_viewer` arg), `camera_and_aurora.launch.py`
  (composes the two — a separate file rather than a toggle on `camera.launch.py`, so either can
  still be run standalone), `inchiscope.launch.py` (full stack), and `record.launch.py`.
- `record.launch.py` runs `rosbag2` against the topic list in `rosbag2/record_topics.yaml`:
  `/camera/image_raw`, `/aurora/sensor_0/pose_relative_to_reference`, `/aurora/reference/pose`,
  `/inchiscope/state` — deliberately excluding high-rate low-level firmware telemetry. Output
  directory defaults to a timestamped name (`inchiscope_bag_<timestamp>`) since `ros2 bag record`
  refuses to overwrite an existing one.
- A saved RViz config (`rviz/aurora.rviz`) opens automatically with `aurora.launch.py`/
  `camera_and_aurora.launch.py`, Fixed Frame set to `aurora_reference` so both tools' poses show
  up already expressed in the bench-fixed world frame.
- See the top-level README's Command reference for the exact commands in current use.

---

## 6. Suggested build/test phasing

1. **Firmware + serial bridge**, bench-tested with a manual CLI (`PISTON d1 45`, `VALVE central
   30`) — get a working, simplified low-level loop before any ROS2 node exists. **Done**, unit
   tested and confirmed on real hardware.
2. **`pba_control_node` + `ab_control_node`** against the bridge — verify CC model and diameter
   mapping against the paper's characterisation data. **Not started** — both are still skeletons
   that abort every goal with a "not yet implemented" message; the CC model and diameter↔pressure
   curve remain to be ported/fit.
3. **`inchiscope_control_node`** state machine + action wiring — this restores full teleoperated
   locomotion, matching current capability but on the new stack. **Not started**, same skeleton
   state as above; blocked on step 2's action servers actually doing something to sequence.
4. **`aurora_tracker_node`** and **`camera_node`** in parallel — these don't block locomotion
   testing and can be developed independently. **Done**, ahead of steps 2-3 as the plan
   anticipated: both are implemented and hardware-verified (Aurora publishing reference/sensor/
   relative pose with RViz visualisation; camera publishing at the corrected 1920x1080 MJPG
   resolution with a working `cv2.imshow` viewer).
5. **`rosbag2` recording + offline reconstruction pipeline** — last, once pose and image streams
   are individually verified. **Recording half done**: `record.launch.py` captures the pose and
   image topics with a timestamped bag name, verified as the correct approach since rosbag2 keeps
   each topic's own message timestamp for later time-correlation. The offline reconstruction
   pipeline itself (combining recorded camera + EM pose data into a 3D reconstruction) has not
   been started.
