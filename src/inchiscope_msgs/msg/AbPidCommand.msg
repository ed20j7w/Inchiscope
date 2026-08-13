# Switches one AB into firmware-side PID pressure-hold mode at the given
# target, overriding any active open-loop VALVE duty for that AB. Firmware
# refuses (ERR) and stays in whatever mode it was in if that AB's own
# pressure sensor wasn't detected at boot -- see PressureState.sensor_connected.
string ab_id                # proximal | central | distal
float32 target_kpa
