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
- **Piston homing is required before commanding a piston.** There are no
  limit switches, so firmware rejects `PISTON` targets for any piston that
  hasn't been sent a `HOME` command yet (blind full-retract, ~30s at the
  default 0-100mm range and current step tunables -- shorter/longer if you
  narrow/widen the range first via `PISTON_RANGE`) -- see
  `src/inchiscope_serial_bridge/README.md`.
- **AB regulators are an I2C DAC** (DFRobot GP8403 at address `0x5F`,
  confirmed from the original pre-ROS2 firmware), not PWM pins -- see
  `src/inchiscope_serial_bridge/README.md`. `REG_RANGE`/`PISTON_RANGE` let
  the PC override the default kPa/mm ranges at runtime if the physical
  hardware ever changes.
- **`AB_PID` (firmware pressure-hold mode) is untested -- do not use yet.**
  It's implemented (`serviceAbPid()` in `main.cpp`) but has never been run
  against real pneumatics: `AB_PID_KP/KI/KD` are unfit placeholder gains,
  and the sign/direction of the PID output -> valve duty -> actual pressure
  response hasn't been verified against a real 3-way valve + regulator pair
  (get that wrong and it drives away from the target instead of toward it).
  Bench-test everything else first via `VALVE` (open-loop, see Command
  reference below); come back to `AB_PID` once open-loop control is
  confirmed working, and expect to iterate on gains and possibly flip a
  sign before trusting it unattended.

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

### Raw serial bench test (no ROS2 required)

Useful for the very first check after flashing -- open a serial monitor at
115200 baud (`pio device monitor -b 115200`, or the Arduino IDE monitor,
newline-terminated) and type these directly. Covers homing, piston range,
and open-loop AB control; **excludes `AB_PID`, which is untested** (see
Open items above) -- don't send it yet.

```
PING                          # expect: ACK PING
PISTON d1 45.0                # expect: ERR piston not homed: d1 (not homed yet)
PISTON_RANGE d1 0 100         # expect: ACK PISTON_RANGE d1 0.000 100.000
HOME d1                       # expect: ACK HOME d1, then watch TEL's d1 homed flag flip 0->1 after ~30s
PISTON d1 45.0                # expect: ACK PISTON d1 45.000, now moves
PISTON_RANGE ALL 10 90        # expect: ACK PISTON_RANGE ALL 10.000 90.000
HOME ALL                      # expect: ACK HOME ALL, homes every connected piston in parallel
PISTON d1 5.0                 # expect: ACK PISTON d1 10.000 (clamped to the new min)
VALVE central 0               # expect: ACK VALVE central 0 (neutral/balanced)
VALVE central 100              # expect: ACK VALVE central 100 (fully toward positive line)
VALVE central -100             # expect: ACK VALVE central -100 (fully toward negative line)
VALVE central 150              # expect: ACK VALVE central 100 (clamped)
REG POS 50                    # expect: ACK REG POS 50.00 (or ERR regulator DAC not connected if the 0x5F DAC isn't wired/detected)
REG NEG -30                   # expect: ACK REG NEG -30.00
REG_RANGE POS 0 80            # expect: ACK REG_RANGE POS 0.00 80.00 -- subsequent REG POS values now map against this range
PISTON x9 10                  # expect: ERR unknown piston id: x9
VALVE nowhere 10              # expect: ERR unknown ab id: nowhere
HOME nowhere                  # expect: ERR unknown home target: nowhere
```

### Home a piston, then command it (required order -- see Open items above)

```bash
ros2 topic pub -1 /firmware/piston_range_cmd inchiscope_msgs/msg/PistonRangeCommand "{id: d1, min_mm: 0.0, max_mm: 100.0}"  # optional, set range before homing
ros2 topic pub -1 /firmware/home_cmd inchiscope_msgs/msg/HomeCommand "{id: d1}"       # or id: ALL
ros2 topic echo /firmware/piston_state --once   # check homed: true before commanding
ros2 topic pub /firmware/piston_cmd inchiscope_msgs/msg/PistonCommand "{id: d1, target_length_mm: 45.0}"
```

### Drive an AB open-loop (PID untested -- see Open items above, don't use `/firmware/ab_pid_cmd` yet)

```bash
ros2 topic pub -1 /firmware/regulator_range_cmd inchiscope_msgs/msg/RegulatorRangeCommand "{regulator_id: positive, min_kpa: 0.0, max_kpa: 100.0}"  # optional, matches firmware defaults
ros2 topic pub -1 /firmware/regulator_cmd inchiscope_msgs/msg/RegulatorCommand "{regulator_id: positive, target_kpa: 80.0}"
ros2 topic pub -1 /firmware/regulator_cmd inchiscope_msgs/msg/RegulatorCommand "{regulator_id: negative, target_kpa: -60.0}"
ros2 topic pub /firmware/valve_cmd inchiscope_msgs/msg/ValveCommand "{ab_id: central, duty_pct: 0.0}"     # neutral/balanced
ros2 topic pub /firmware/valve_cmd inchiscope_msgs/msg/ValveCommand "{ab_id: central, duty_pct: 100.0}"   # fully toward positive line
ros2 topic pub /firmware/valve_cmd inchiscope_msgs/msg/ValveCommand "{ab_id: central, duty_pct: -100.0}"  # fully toward negative line
ros2 topic echo /firmware/pressure_state   # check mode (should stay open_loop) and sensor_connected
```

### Inspect topics / tf

```bash
ros2 topic list
ros2 topic echo /firmware/piston_state
ros2 topic echo /aurora/sensor_0/pose_relative_to_reference
ros2 topic hz /aurora/sensor_0/pose
ros2 topic hz /camera/image_raw
ros2 run tf2_ros tf2_echo aurora_field aurora_reference
```

See `src/inchiscope_aurora/README.md` for the Aurora vendor SDK / `.rom`
file setup, `src/inchiscope_camera/README.md` for the camera viewer, and
`src/inchiscope_bringup/rviz/README.md` for the saved RViz config.
