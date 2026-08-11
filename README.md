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
  building. The reference tool's SROM is on its own physical chip and is
  auto-detected; only the 6D sensor needs a `.rom` file (its virtual SROM)
  -- set `sensor_srom_path` in `inchiscope_bringup/config/params.yaml`. See
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
```

ROS2 workspace build commands are in the command reference below.

## Command reference

Running log of the commands actually used to build/run/debug this stack --
added to as new ones come up, so it doubles as a working history rather
than a one-time cheat sheet.

### Build

```bash
colcon build --symlink-install              # normal dev build -- edits to yaml/launch files take
                                             # effect without rebuilding
colcon build --packages-select <pkg_name>   # rebuild just one package
```

If colcon fails with `Failed to create symbolic link '...' because
existing path cannot be removed: Is a directory` -- stale build artifacts,
typically from a package's build type changing (e.g. ament_python ->
ament_cmake) or mixing symlink/non-symlink builds:

```bash
rm -rf build/<pkg_name> install/<pkg_name>  # targeted clean, try this first
rm -rf build install log                    # full clean, if targeted doesn't fix it
```

`inchiscope_aurora` needs the vendor SDK at
`third_party/ndi_combined_api/CombinedAPIsample/` (repo root, sibling of
`src/`, gitignored); if you keep it elsewhere:

```bash
colcon build --packages-select inchiscope_aurora \
  --cmake-args -DNDI_COMBINED_API_DIR=/path/to/CombinedAPIsample
```

### Source

```bash
source install/setup.bash
```

### Launch

```bash
ros2 launch inchiscope_bringup phase1_bridge.launch.py       # firmware + serial bridge only
ros2 launch inchiscope_bringup aurora.launch.py              # Aurora tracker + RViz (use_rviz:=false to skip)
ros2 launch inchiscope_bringup camera.launch.py              # camera + cv2 viewer (show_viewer:=false to skip)
ros2 launch inchiscope_bringup camera_and_aurora.launch.py   # camera + viewer + Aurora tracker + RViz together
ros2 launch inchiscope_bringup inchiscope.launch.py          # full stack (use_rviz:=true to also open RViz)
ros2 launch inchiscope_bringup record.launch.py              # rosbag2 record -> ./inchiscope_bag_<timestamp>/ (bag_name:=... to override)
```

### Run a single node directly

```bash
ros2 run inchiscope_serial_bridge serial_bridge_node --ros-args --params-file src/inchiscope_bringup/config/params.yaml
ros2 run inchiscope_aurora aurora_tracker_node --ros-args --params-file src/inchiscope_bringup/config/params.yaml
ros2 run inchiscope_camera camera_node --ros-args --params-file src/inchiscope_bringup/config/params.yaml
ros2 run inchiscope_camera camera_viewer_node
```

### Inspect topics / tf

```bash
ros2 topic list
ros2 topic echo /firmware/piston_state
ros2 topic echo /aurora/sensor_0/pose_relative_to_reference
ros2 topic hz /aurora/sensor_0/pose
ros2 topic hz /camera/image_raw
ros2 topic pub /firmware/piston_cmd inchiscope_msgs/msg/PistonCommand "{id: d1, target_length_mm: 45.0}"
ros2 run tf2_ros tf2_echo aurora_field aurora_reference
```

See `src/inchiscope_aurora/README.md` for the Aurora vendor SDK / `.rom`
file setup, `src/inchiscope_camera/README.md` for the camera viewer, and
`src/inchiscope_bringup/rviz/README.md` for the saved RViz config.
