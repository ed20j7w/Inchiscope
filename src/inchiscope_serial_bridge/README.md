# inchiscope_serial_bridge

Owns the serial link to the Mega and is a pure translation layer -- no
kinematics, no calibration math here (see `inchiscope_ros2_architecture.md`
section 5). This is also where the ASCII wire protocol lives
(`inchiscope_serial_bridge/protocol.py`, unit tested independent of ROS2/
hardware in `test/test_protocol.py`).

## Piston homing

There are no limit switches on this hardware, so a piston's absolute
position is only known once it's been homed: driven fully retracted for
long enough that even a piston starting at its configured max is guaranteed
to have reached the mechanical bottom (with margin for stall/slip), then
zeroed to its configured min. Firmware refuses `PISTON` targets for any
piston that hasn't been homed yet -- publish a `HomeCommand` on
`/firmware/home_cmd` first (`id` = a specific piston id, or `ALL`), and
check `PistonState.homed` on `/firmware/piston_state` before commanding that
piston. Homing runs in the firmware's own timer, not blocking the serial
link or telemetry -- other pistons keep servoing/reporting while one homes.

Each piston's usable travel defaults to the full 0-100mm stroke and is
settable via `PistonRangeCommand` on `/firmware/piston_range_cmd` (specific
piston or `ALL`). This isn't just a clamp: the next `HOME` for that piston
computes its blind-retract duration from whatever range is configured at
that moment, so narrowing the range also shortens homing, and widening it
lengthens homing -- set the range you want *before* homing, not after.

## AB pressure control

Each AB (anchoring balloon) is driven by a single 3-way valve switching
between two **shared** analog pressure regulators (positive/inflate,
negative/vacuum). The regulators are driven over **I2C**, not PWM: a
DFRobot GP8403 2-channel 0-10V DAC at address `0x5F` (channel 0 = positive,
channel 1 = negative), confirmed from the original pre-ROS2 firmware. They
are open-loop from the firmware's point of view -- they're proportional
hardware that self-regulates off the DAC voltage it's given, so there's no
pressure feedback on those two lines, only on the three AB-side sensors.
Set them via `RegulatorCommand` on `/firmware/regulator_cmd`
(`regulator_id`: `positive`/`negative`). The kPa<->voltage mapping range for
each regulator defaults to its physical limits (0..100 kPa positive,
-100..0 kPa negative) and is settable via `RegulatorRangeCommand` on
`/firmware/regulator_range_cmd`, same idea as `PistonRangeCommand` above.

Per AB, there are two mutually exclusive ways to command it, matching
whichever command was sent last:

- **Open loop** -- publish `ValveCommand` on `/firmware/valve_cmd`:
  `duty_pct` is a signed -100..100 3-way valve position (+100 fully toward
  the positive line, -100 fully toward negative). Always switches that AB to
  open-loop mode.
- **PID hold** -- publish `AbPidCommand` on `/firmware/ab_pid_cmd` with a
  `target_kpa`. Runs a PID loop **on the Mega** (100 Hz, independent of the
  20 Hz telemetry rate) against that AB's own pressure sensor, converting
  the output into the same signed valve-duty representation as the open-loop
  path. Refused with an `ERR` (mode doesn't change) if that AB's pressure
  sensor wasn't detected at boot -- check `PressureState.sensor_connected`
  on `/firmware/pressure_state` before relying on this.
  **Untested -- do not use yet.** `AB_PID_KP/KI/KD` are unfit placeholder
  gains, and the PID-output-to-actual-pressure sign convention hasn't been
  verified against a real 3-way valve + regulator pair (wrong sign drives
  away from the target, not toward it). Confirm open-loop `VALVE` control
  works correctly first; treat `AB_PID` as needing bench iteration on gains
  and possibly a sign flip before it's trustworthy.

`PressureState.mode` (`"open_loop"` | `"pid"`) is ground truth reported by
firmware, not something the bridge or PC infers -- it reflects whichever
command firmware actually accepted last.

## Topics

- Out: `/firmware/piston_state` (`PistonStateArray`, includes `homed`),
  `/firmware/pressure_state` (`PressureStateArray`, includes `mode` and
  `sensor_connected`).
- In: `/firmware/piston_cmd` (`PistonCommand`), `/firmware/home_cmd`
  (`HomeCommand`), `/firmware/piston_range_cmd` (`PistonRangeCommand`),
  `/firmware/valve_cmd` (`ValveCommand`), `/firmware/ab_pid_cmd`
  (`AbPidCommand`), `/firmware/regulator_cmd` (`RegulatorCommand`),
  `/firmware/regulator_range_cmd` (`RegulatorRangeCommand`).

See `firmware/inchiscope_mega/src/main.cpp` for the wire-level command/
telemetry format and `inchiscope_ros2_architecture.md` section 3 for the
full protocol reference.
