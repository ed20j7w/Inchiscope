# inchiscope_aurora

`aurora_tracker_node` connects to the NDI Aurora field generator over a
serial port, loads/detects two tools, starts tracking, and publishes both
tools' pose:

- **reference** -- its SROM is on its own physical chip, so it's
  auto-detected once the node connects: no `.rom` file or path needed for
  it at all.
- **sensor_0** -- the 6D sensor mounted at the endoscope tip, identified by
  a **virtual SROM**: a tool definition file uploaded over the wire instead
  of read from a physical connector chip, since the bare sensor coil has no
  onboard SROM chip of its own. NDI (or whoever characterised your coil)
  will have given you this as a `.rom` file.

Topics: `/aurora/reference/pose`, `/aurora/sensor_0/pose`
(`geometry_msgs/msg/PoseStamped`, in the `aurora_field` frame by default),
plus matching tf2 transforms.

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

The node refuses to connect (and logs why, throttled) until this path is
set and readable. Nothing needs setting for the reference tool -- it's
found automatically once connected.

## 3. Run it

```bash
ros2 run inchiscope_aurora aurora_tracker_node --ros-args --params-file <path-to-params.yaml>
```

It retries the connect/load/init/enable/start-tracking sequence on a timer
(`reconnect_period_sec`, default 5s) until it succeeds, so it's safe to
launch before the Aurora unit is powered on.

## Known simplifications

- Uses the classic binary `BX` command, not `BX2` -- Aurora doesn't support
  `BX2` (Vega/Polaris-only), so this is the correct choice.
- Publishes each tool's raw pose in the field generator's frame. It does
  **not** compute a reference-compensated pose (sensor pose expressed
  relative to the moving reference tool, i.e.
  `reference_pose.inverse() * sensor_pose`) -- that's a straightforward
  follow-up if you need pose stable against patient/field-generator motion,
  but nothing currently computes it.
- No TLS/DTLS support (not needed for a local serial connection to Aurora,
  and not built -- see the CMakeLists.txt comment on `TlsConnection.cpp`).
