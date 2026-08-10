# Inchiscope

ROS2 (Jazzy, Ubuntu 24.04) control stack for a soft-robotic endoscope, plus EM
tracker + camera integration for offline 3D reconstruction. Full design spec:
[`inchiscope_ros2_architecture.md`](inchiscope_ros2_architecture.md).

## Layout

```
firmware/
└── inchiscope_mega/        # PlatformIO project, Arduino Mega, servo-loop only
src/
├── inchiscope_msgs/           # rosidl interfaces (msg + action)
├── inchiscope_serial_bridge/  # PC <-> Mega translation layer            [Phase 1 - done]
├── inchiscope_pba_control/    # CC model + piston targets                [Phase 2 - skeleton]
├── inchiscope_ab_control/     # diameter -> pressure, safety clamp       [Phase 2 - skeleton]
├── inchiscope_control/        # state machine, action orchestration      [Phase 3 - skeleton]
├── inchiscope_aurora/         # EM tracker pose (C++/ament_cmake)        [Phase 4 - implemented, needs vendor SDK + .rom files]
├── inchiscope_camera/         # NanEye feed via capture card             [Phase 4 - functional]
└── inchiscope_bringup/        # launch files, params, rosbag2 recording
```

## Status

**Phase 1 (firmware + serial bridge) is implemented and unit tested.**
Everything else is deliberately scaffolded, not faked: nodes start, wire up
their topics/actions/params, and any action goal that needs real control
logic aborts with an explanatory message pointing at the phase that
implements it. See `inchiscope_ros2_architecture.md` section 6 for the
intended build order.

### Open items

- **Aurora EM tracker.** `inchiscope_aurora` is now a real ament_cmake C++
  node (`aurora_tracker_node`) built directly against NDI's "Combined API
  Sample C++ v1.9.7" -- no pybind11 needed, since the SDK's `connect()`
  handles a serial device path natively. The SDK itself is still **not
  vendored into this repo** (proprietary, no redistribution grant in its
  `license.txt`): drop it locally under
  `third_party/ndi_combined_api/CombinedAPIsample/` (gitignored) before
  building, and set `reference_srom_path` / `sensor_srom_path` in
  `inchiscope_bringup/config/params.yaml` to your actual `.rom` files. See
  `src/inchiscope_aurora/README.md` for the full setup.
- **AB diameter->pressure curve and pressure ceilings** in
  `inchiscope_bringup/config/params.yaml` are placeholders pending the real
  fit from the paper's characterisation data (Fig. 4a-I, section III-A).
- **Hardware mapping in firmware**: the current bench hardware only has one
  PBA segment's worth of piston actuators wired (3 steppers). The new
  protocol reserves ids for both segments (`p1..p3`, `d1..d3`); firmware
  maps the wired steppers to `d1..d3` and treats `p1..p3` as not-connected
  until the second segment is built. See the pin-mapping comment at the top
  of `firmware/inchiscope_mega/src/main.cpp` if that assumption is wrong for
  your bench setup.

## Build

```bash
# firmware
cd firmware/inchiscope_mega && pio run -t upload

# ROS2 workspace
colcon build --symlink-install
source install/setup.bash
```

## Bench-test phase 1

```bash
ros2 launch inchiscope_bringup phase1_bridge.launch.py
ros2 topic pub /firmware/piston_cmd inchiscope_msgs/msg/PistonCommand "{id: d1, target_length_mm: 45.0}"
ros2 topic echo /firmware/piston_state
```
