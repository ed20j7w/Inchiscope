# Inchiscope ROS2 Architecture

Target platform: **ROS2 Jazzy**, Ubuntu 24.04, Python-first (`rclpy`) for all logic nodes.
Firmware: **PlatformIO** on Arduino Mega, C++, servo-loop only (no kinematics, no calibration math).

This document is the handoff spec for implementation in Claude Code. It defines the workspace
layout, the serial protocol between PC and Mega, the custom ROS2 interfaces, the node
responsibilities, and the build/test phasing agreed in planning.

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
    ├── inchiscope_aurora/          # ament_python (+ vendor SDK wrapper if needed) — pose
    ├── inchiscope_camera/          # ament_python (+ vendor driver wrapper if needed) — image
    └── inchiscope_bringup/         # launch files, params, rosbag2 recording config
```

Interface generation (`.msg`/`.action`) requires `rosidl`, which needs a small `ament_cmake`
package (`inchiscope_msgs`) even though everything else is Python — this is normal for ROS2 and
is the only non-Python package in the workspace.

**Open item:** the Aurora and NanEye SDKs are typically C/C++. If no Python bindings exist,
`inchiscope_aurora` / `inchiscope_camera` may need a thin pybind11 or ctypes wrapper around a
vendor `.so`/driver rather than pure Python. Confirm SDK language before implementation — flag
to Claude Code as a spike/investigation task, not an assumption.

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

### `aurora_tracker_node` (inchiscope_aurora)
- Generates/writes the virtual SROM at startup, opens the Aurora field generator connection,
  publishes `PoseStamped` on `/aurora/sensor_0/pose` and broadcasts the corresponding `tf2`
  transform. Namespaced for a future `sensor_1`, `sensor_2`.

### `camera_node` (inchiscope_camera)
- Publishes the NanEye feed on `/camera/image_raw` (+ `/camera/camera_info` if/when calibrated).

### `inchiscope_bringup`
- Launch files composing the above, parameter YAML (pressure ceilings, port names, curve
  coefficients), and a `rosbag2` recording profile capturing at minimum:
  `/camera/image_raw`, `/aurora/sensor_0/pose`, `/inchiscope/state` — enough for offline
  reconstruction without bloating the bag with high-rate low-level telemetry.

---

## 6. Suggested build/test phasing

1. **Firmware + serial bridge**, bench-tested with a manual CLI (`PISTON d1 45`, `VALVE central
   30`) — get a working, simplified low-level loop before any ROS2 node exists.
2. **`pba_control_node` + `ab_control_node`** against the bridge — verify CC model and diameter
   mapping against the paper's characterisation data.
3. **`inchiscope_control_node`** state machine + action wiring — this restores full teleoperated
   locomotion, matching current capability but on the new stack.
4. **`aurora_tracker_node`** and **`camera_node`** in parallel — these don't block locomotion
   testing and can be developed independently.
5. **`rosbag2` recording + offline reconstruction pipeline** — last, once pose and image streams
   are individually verified.
