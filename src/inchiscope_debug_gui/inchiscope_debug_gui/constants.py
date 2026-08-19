"""Firmware-fixed identifiers and defaults.

Mirrors firmware/inchiscope_mega/src/main.cpp -- these are hardcoded array
sizes on the Mega, not something the bridge or firmware exposes discovery
for, so the GUI hardcodes them too rather than trying to query them.
"""

PISTON_IDS = ['p1', 'p2', 'p3', 'd1', 'd2', 'd3']

# PISTON_CONNECTED in main.cpp -- only the distal segment is wired on the
# current bench setup; p1-p3 are reserved for a second segment not yet
# installed. Commands to an unconnected piston are just ignored by firmware,
# so the GUI greys them out rather than letting you command a no-op.
PISTON_CONNECTED = {
    'p1': False, 'p2': False, 'p3': False,
    'd1': True, 'd2': True, 'd3': True,
}

AB_IDS = ['proximal', 'central', 'distal']

REGULATOR_IDS = ['positive', 'negative']

# Defaults the firmware itself boots with (DEFAULT_PISTON_MIN_MM/MAX_MM,
# DEFAULT_PISTON_SPEED_MM_S, DEFAULT_REG_POS_MAX_KPA/REG_NEG_MIN_KPA in
# main.cpp). The GUI has no readback for configured range/speed/regulator
# range (firmware doesn't report them in telemetry), so these are only used
# to seed slider bounds at startup -- they go stale the moment you Apply a
# different range/speed and are not re-synced from firmware afterwards.
DEFAULT_PISTON_MIN_MM = 0.0
DEFAULT_PISTON_MAX_MM = 100.0
DEFAULT_PISTON_SPEED_MM_S = 10.0
MIN_PISTON_SPEED_MM_S = 0.1
MAX_PISTON_SPEED_MM_S = 120.0

DEFAULT_REG_POS_MAX_KPA = 100.0
DEFAULT_REG_NEG_MIN_KPA = -100.0

VALVE_DUTY_MIN_PCT = -100.0
VALVE_DUTY_MAX_PCT = 100.0

# How often armed, touched controls re-publish their current value (Hz).
# Not tied to slider drag events -- see ros_bridge.py's heartbeat timer.
HEARTBEAT_HZ = 15.0

# If no telemetry arrives for this long, the GUI auto-disarms and flags the
# bridge as disconnected. Matches serial_bridge_node's own
# telemetry_timeout_sec default so the GUI trips at the same point the
# bridge's own watchdog log warning does.
TELEMETRY_TIMEOUT_SEC = 1.0
