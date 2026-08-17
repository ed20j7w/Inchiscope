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
├── inchiscope_reconstruction/ # offline mesh from a bag (COLMAP+Open3D)  [Phase 5 - stage 1-2 implemented]
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
  hasn't been sent a `HOME` command yet (blind full-retract, ~15s at the
  default 0-100mm range and 10mm/s default speed -- shorter/longer if you
  narrow/widen the range via `PISTON_RANGE` or change speed via
  `PISTON_SPEED` first) -- see `src/inchiscope_serial_bridge/README.md`.
- **AB regulators are an I2C DAC** (DFRobot GP8403 at address `0x5F`,
  confirmed from the original pre-ROS2 firmware), not PWM pins -- see
  `src/inchiscope_serial_bridge/README.md`. `REG_RANGE`/`PISTON_RANGE` let
  the PC override the default kPa/mm ranges at runtime if the physical
  hardware ever changes.
- **Piston speed is runtime-settable** (`PISTON_SPEED`, mm/s), default
  `10mm/s`. **Bench-confirmed: this actuator doesn't move reliably at all
  from 15mm/s up** (a clean threshold, not gradual degradation) -- the
  published Actuonix S20-38 full-step load curve alone doesn't explain
  that, and it isn't caused by I2C blocking from the AB pressure reads
  (ruled out: those sensors aren't wired on this bench setup, and
  `readAbPressures()` already skips undetected sensors). Most likely cause,
  not yet fixed: no acceleration ramp on start -- the datasheet warns full
  step "will not move" at higher speeds without one. See
  `src/inchiscope_serial_bridge/README.md`.
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
- **Camera crop set, calibrated, and `/camera/camera_info` now published.**
  The NanEye sensor's real content is only ~320x320; the capture card pads
  that with a black border (plus a logo/info overlay in part of it) out to
  whichever resolution is requested. **Bench-confirmed: requesting a
  non-16:9 size (`640x480`, this device's smallest overall) made the
  squash worse, not better** -- presumably this ISP only pads/scales
  correctly for a 16:9 target. Capture resolution is `1280x720` (the
  smallest exact-16:9 size this device offers); `camera_node`'s
  `crop_x/y/width/height` are set in `params.yaml` (`391,111,480,480`) and
  confirmed to publish a clean 480x480 square with no border or logo.
  Calibrated against that exact crop (8mm-square board, 27/27 frames,
  0.55px RMS reprojection error) -- see `camera_info_path` in
  `params.yaml` and `src/inchiscope_camera/scripts/README.md`.
  `camera_node` cross-checks the calibration file's resolution against
  what it's actually publishing and refuses to publish `/camera/camera_info`
  (logging why) on a mismatch, e.g. after a resolution/crop change without
  recalibrating. See `src/inchiscope_camera/README.md` for how to
  re-measure the crop if this ever needs redoing (different capture-card
  hardware, or a resolution change).
- **Camera-to-EM-sensor transform is a zero-offset `PLACEHOLDER`.**
  `camera_and_aurora.launch.py` and `inchiscope.launch.py` publish a static
  transform from `aurora_sensor_0` to `naneye_camera` so RViz's `Camera`
  display (if used instead of the simpler `Image` display) can resolve the
  image's frame at all -- without *some* transform it errors with `Frame
  [naneye_camera] does not exist`. The camera and 6D EM sensor are mounted
  together at the distal tip (per the manuscript) but their exact rigid
  offset has never been measured -- treating them as co-located is wrong,
  just less broken than not publishing anything. Replace with the real
  offset once a proper hand-eye/mechanical calibration is done; this
  matters for reconstruction accuracy too, not just RViz, since it's the
  offset between where the EM tracker says the tip is and where the camera
  that's actually seeing the scene sits.

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
PISTON_SPEED d1 10            # expect: ACK PISTON_SPEED d1 10.000 -- also the current default, so a no-op here
HOME d1                       # expect: ACK HOME d1, then watch TEL's d1 homed flag flip 0->1 after ~15s at 10mm/s
PISTON d1 45.0                # expect: ACK PISTON d1 45.000, now moves
PISTON_SPEED d1 5             # expect: ACK PISTON_SPEED d1 5.000 -- slower, more force margin
PISTON d1 20.0                # expect: ACK PISTON d1 20.000, visibly slower than the first move
# don't go above ~15mm/s -- bench-confirmed the actuator doesn't move reliably at all up there, see Open items
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
ros2 topic pub -1 /firmware/piston_speed_cmd inchiscope_msgs/msg/PistonSpeedCommand "{id: d1, mm_per_s: 10.0}"  # optional, set speed before homing (also default) -- don't go above ~15mm/s, see Open items
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
