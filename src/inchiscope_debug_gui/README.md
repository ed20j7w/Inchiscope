# inchiscope_debug_gui

A standalone PyQt5 debug interface that talks directly to every
`/firmware/*` topic `inchiscope_serial_bridge` exposes -- regulators, AB
valves, AB_PID, and pistons -- with a slider + numeric box for every
settable value. It bypasses `inchiscope_ab_control` / `inchiscope_pba_control`
entirely: there's no arbitration between this GUI and those nodes, so don't
run both against the same hardware at once (see "Bypassing the higher-level
controllers" below).

## Running it

```bash
ros2 launch inchiscope_bringup phase1_bridge.launch.py   # brings up serial_bridge_node only
ros2 run inchiscope_debug_gui debug_gui
```

`phase1_bridge.launch.py` is exactly the "nothing but the bridge" bringup
this GUI is meant to sit on top of.

## Layout

- **Regulators & Duty Cycles**
  - *Pressure Regulators* -- `positive`/`negative` target-kPa sliders
    (`/firmware/regulator_cmd`), plus a Range config (`/firmware/regulator_range_cmd`)
    that rescales the slider bounds to whatever you last applied. There's no
    feedback sensor on these lines, so there's nothing to read back --
    what you see is only ever what was last commanded from this GUI.
  - *AB Valves* -- `proximal`/`central`/`distal` open-loop duty sliders
    (`/firmware/valve_cmd`, -100=vacuum..+100=inflate). Sending one always
    forces that AB into open-loop mode on the firmware.
  - *AB_PID (experimental)* -- hidden behind an explicit "I understand this
    is untested" checkbox. `inchiscope_serial_bridge/README.md` flags the
    firmware's PID gains as unverified placeholders; this GUI treats
    setting a PID target as a one-shot button press, never a streamed
    slider, to keep it from re-triggering repeatedly while you drag.
- **Actuators** -- one card per piston (`p1-p3` reserved/greyed out, not
  wired on the current bench setup; `d1-d3` live), each with a live
  length/homed/at-target readout, a target-length slider
  (`/firmware/piston_cmd`, disabled until `homed=true` -- mirrors
  firmware's own rejection of PISTON before HOME instead of just letting
  every send silently error), a Home button (`/firmware/home_cmd`, behind a
  confirmation dialog since it's a blind full-retract), and Range/Speed
  config (`/firmware/piston_range_cmd` / `/firmware/piston_speed_cmd`).
- **Pressure Readings** -- read-only: live pressure, sensor-connected
  status, mode, and (when in `pid` mode) target/at-target for each AB.

## Safety model

- **Master ARM toggle.** Every outgoing command -- heartbeat-streamed or
  button-triggered -- is refused while disarmed. The GUI starts disarmed.
- **Touch-gating, not just ARM-gating.** Regulator and valve controls only
  join the heartbeat once *you've actually moved that specific control*
  since the GUI started. There's no way to read back the regulators' or
  valves' true current state, so the alternative -- streaming a
  default/zero value to every control the instant you hit ARM -- could
  silently override a setpoint someone already dialled in by hand before
  you connected. Piston targets are the one exception: they're synced from
  the piston's own reported `target_length_mm` the first time telemetry
  arrives, so including them in the heartbeat immediately is a no-op
  against the real hardware, not a surprise change.
- **Heartbeat, not drag events.** While armed, every touched control
  re-publishes its current value at a fixed 15 Hz (`constants.HEARTBEAT_HZ`)
  regardless of whether you're actively dragging. This is simpler and more
  robust than trying to detect "currently dragging" per widget, and it
  self-heals a dropped serial line the same way holding a joystick does.
- **Auto-disarm on stale telemetry.** If no telemetry arrives for longer
  than `constants.TELEMETRY_TIMEOUT_SEC` (1.0s, matching the bridge's own
  `telemetry_timeout_sec` default) while armed, the GUI disarms itself and
  flags the bridge status as stale.
- **Home requires confirmation.** Both the per-piston Home button and Home
  All show a confirmation dialog first, since homing blind-retracts for a
  fixed duration with no position feedback until it completes.

## Bypassing the higher-level controllers

There is no mode flag anywhere in the stack that arbitrates between this
GUI and `inchiscope_ab_control` / `inchiscope_pba_control` -- whichever one
last published to a given `/firmware/*_cmd` topic wins, full stop. As of
this writing those controller packages aren't running against real
hardware yet, so there's nothing to actually conflict with, but if that
changes: don't run this GUI and either controller against the same
hardware at the same time, since their commands will just stomp on each
other with no warning from firmware or bridge.

## Known limitations

- No readback exists for configured piston range/speed or regulator range
  -- the GUI's "Configured range" fields only reflect what *this GUI*
  session has applied, seeded from the firmware's own boot defaults
  (0-100mm / 10mm/s / 0-100kPa / -100-0kPa). If you restart the GUI without
  restarting firmware, its range fields reset to those defaults even though
  the firmware may still be running whatever was last applied before the
  restart.
- No readback exists for regulator target or valve duty either -- the
  "touch before heartbeat" gating (above) is the mitigation, not a full
  substitute for a real readback.
- This was built and exercised against a fake in-process rclpy/PyQt5 stub
  (see the smoke test used during development) rather than real hardware
  or a real ROS2 install -- treat first use on the bench as the real test
  of the serial round-trip, same as any other new bridge client.

## Tests

```bash
cd src/inchiscope_debug_gui
python3 -m pytest test/
```

Only `scaling.py` (the float<->slider-int mapping) is pure-logic and
covered here; everything else depends on PyQt5 and rclpy, neither of which
is available in this repo's dev sandbox.
