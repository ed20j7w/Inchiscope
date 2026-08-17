# inchiscope_aurora

`aurora_tracker_node` connects to the NDI Aurora field generator over a
serial port, auto-detects both tools, starts tracking, and publishes both
tools' pose:

- **reference** -- its SROM is on its own physical chip. No `.rom` file,
  path, or port number needed for it at all.
- **sensor_0** -- the 6D sensor mounted at the endoscope tip, identified by
  a **virtual SROM**: a tool definition file uploaded over the wire instead
  of read from a physical connector chip, since the bare sensor coil has no
  onboard SROM chip of its own. NDI (or whoever characterised your coil)
  will have given you this as a `.rom` file.

Neither tool needs you to say which physical SCU port it's on. Aurora
doesn't have a `PHRQ` command at all (checked against the official API
guide's command list -- it's Polaris/Vega-only terminology for a tool with
no physical connection, which doesn't describe an Aurora sensor); instead,
a plain port search (`PHSR`) auto-detects and assigns handles to both
tools, chip or not, since every Aurora tool is physically wired to a
numbered port. The node tells them apart by whether a port handle already
has chip data (the reference) or not (the sensor, which then gets its
virtual SROM uploaded onto that handle) -- so it doesn't matter which
physical port either tool is wired into.

Topics (all `geometry_msgs/msg/PoseStamped`):

- `/aurora/reference/pose`, `/aurora/sensor_0/pose` -- raw poses in the
  field generator's own frame (`aurora_field` by default), plus matching
  tf2 transforms.
- `/aurora/sensor_0/pose_relative_to_reference` -- the sensor's pose
  composed into the reference tool's frame
  (`T_reference_to_sensor = T_field_to_reference⁻¹ · T_field_to_sensor`),
  published in `reference_frame_id` (`aurora_reference` by default). Since
  the reference sits flat on the table, this is effectively the sensor's
  pose in a bench-fixed world frame -- this is the one to record for
  reconstruction, since it cancels out the field generator's arbitrary
  internal frame (and any small reference-tool drift) without downstream
  consumers needing to redo a tf lookup themselves. Only published when
  both tools are in view this cycle (skipped if either is missing).

## 1. Get the vendor SDK

This package links directly against NDI's "Combined API Sample C++" SDK.
It is **not vendored into this repo**: it's proprietary NDI sample code with
no redistribution grant in its `license.txt`. Extract the vendor zip
locally so that this path exists:

```
<repo root>/third_party/ndi_combined_api/CombinedAPIsample/library/include/CombinedApi.h
```

(`third_party/` is gitignored.) If you'd rather keep it somewhere else,
point CMake at it explicitly:

```bash
colcon build --packages-select inchiscope_aurora \
  --cmake-args -DNDI_COMBINED_API_DIR=/path/to/CombinedAPIsample
```

If the SDK isn't found, `colcon build` prints a warning and skips
`aurora_tracker_node` rather than failing the whole workspace -- check the
build log if the executable is missing.

## 2. Get the sensor's virtual SROM

Put the 6D sensor's `.rom` file somewhere findable -- e.g.
`inchiscope_bringup/config/sroms/` (see the README there) -- and set it
before launching (`inchiscope_bringup/config/params.yaml` or
`--ros-args -p ...`):

```yaml
aurora_tracker_node:
  ros__parameters:
    field_generator_port: /dev/ttyUSB0
    sensor_srom_path: /path/to/distal_sensor_virtual.rom
```

The node refuses to connect (and logs why, throttled) until `sensor_srom_path`
is set and readable. Nothing needs setting for the reference tool -- it's
found automatically once connected, wherever it's plugged in.

## 3. Run it

```bash
ros2 run inchiscope_aurora aurora_tracker_node --ros-args --params-file <path-to-params.yaml>
```

It retries the connect/load/init/enable/start-tracking sequence on a timer
(`reconnect_period_sec`, default 5s) until it succeeds, so it's safe to
launch before the Aurora unit is powered on.

## Measuring the tracker's own noise floor

`scripts/measure_aurora_noise.py` -- standalone, not a ROS node registered
with the package -- holds the sensor still and reports how much
`/aurora/sensor_0/pose_relative_to_reference` jitters on its own, in
position (mm) and orientation (degrees). Run it with `aurora_tracker_node`
already up and the sensor resting on something solid (not free-handed --
that adds tremor that isn't the tracker's own noise):

```bash
python3 scripts/measure_aurora_noise.py --duration-sec 15
```

This isolates the tracker/environment's own accuracy from everything else
in the pipeline (capture timing, hand-eye calibration, camera intrinsics).
Compare its numbers directly against the checkerboard-in-reference spread
`inchiscope_camera/scripts/calibrate_hand_eye.py solve` reports -- if
they're already close, no amount of better capture technique or
recalibration elsewhere will close the remaining gap; the tracker itself
(or nearby metal, or distance from the field generator at that bench
location) is the limiting factor.

## Known simplifications

- Uses the classic binary `BX` command, not `BX2` -- Aurora doesn't support
  `BX2` (Vega/Polaris-only), so this is the correct choice.
- No TLS/DTLS support (not needed for a local serial connection to Aurora,
  and not built -- see the CMakeLists.txt comment on `TlsConnection.cpp`).
