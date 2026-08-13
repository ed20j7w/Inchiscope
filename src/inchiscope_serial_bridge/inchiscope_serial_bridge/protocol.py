"""Pure encode/decode helpers for the PC<->Mega ASCII serial protocol.

Kept free of rclpy/pyserial so the wire format can be unit tested without
hardware or a ROS2 environment. See inchiscope_ros2_architecture.md section 3
for the protocol definition.
"""

from dataclasses import dataclass

PISTON_IDS = ('p1', 'p2', 'p3', 'd1', 'd2', 'd3')
AB_IDS = ('proximal', 'central', 'distal')
REGULATOR_IDS = ('positive', 'negative')

_AB_MODE_WIRE_TO_NAME = {'O': 'open_loop', 'P': 'pid'}


def format_piston_command(piston_id: str, target_length_mm: float) -> str:
    if piston_id not in PISTON_IDS:
        raise ValueError(f'unknown piston id: {piston_id}')
    return f'PISTON {piston_id} {target_length_mm:.3f}'


def format_home_command(target: str) -> str:
    if target != 'ALL' and target not in PISTON_IDS:
        raise ValueError(f'unknown home target: {target}')
    return f'HOME {target}'


def format_piston_range_command(target: str, min_mm: float, max_mm: float) -> str:
    if target != 'ALL' and target not in PISTON_IDS:
        raise ValueError(f'unknown piston range target: {target}')
    if min_mm >= max_mm:
        raise ValueError(f'min_mm ({min_mm}) must be < max_mm ({max_mm})')
    return f'PISTON_RANGE {target} {min_mm:.3f} {max_mm:.3f}'


def format_piston_speed_command(target: str, mm_per_s: float) -> str:
    if target != 'ALL' and target not in PISTON_IDS:
        raise ValueError(f'unknown piston speed target: {target}')
    if mm_per_s <= 0:
        raise ValueError(f'mm_per_s ({mm_per_s}) must be > 0')
    return f'PISTON_SPEED {target} {mm_per_s:.3f}'


def format_valve_command(ab_id: str, duty_pct: float) -> str:
    """Open-loop 3-way valve position. Always switches this AB to open-loop
    mode on the firmware, overriding any active AB_PID hold."""
    if ab_id not in AB_IDS:
        raise ValueError(f'unknown ab id: {ab_id}')
    duty_pct = max(-100, min(100, int(round(duty_pct))))
    return f'VALVE {ab_id} {duty_pct}'


def format_ab_pid_command(ab_id: str, target_kpa: float) -> str:
    """Switches this AB to firmware-side PID pressure hold at target_kpa."""
    if ab_id not in AB_IDS:
        raise ValueError(f'unknown ab id: {ab_id}')
    return f'AB_PID {ab_id} {target_kpa:.2f}'


def format_regulator_command(regulator_id: str, target_kpa: float) -> str:
    if regulator_id not in REGULATOR_IDS:
        raise ValueError(f'unknown regulator id: {regulator_id}')
    token = 'POS' if regulator_id == 'positive' else 'NEG'
    return f'REG {token} {target_kpa:.2f}'


def format_regulator_range_command(regulator_id: str, min_kpa: float, max_kpa: float) -> str:
    if regulator_id not in REGULATOR_IDS:
        raise ValueError(f'unknown regulator id: {regulator_id}')
    if min_kpa >= max_kpa:
        raise ValueError(f'min_kpa ({min_kpa}) must be < max_kpa ({max_kpa})')
    token = 'POS' if regulator_id == 'positive' else 'NEG'
    return f'REG_RANGE {token} {min_kpa:.2f} {max_kpa:.2f}'


def format_ping() -> str:
    return 'PING'


@dataclass
class Telemetry:
    t_ms: int
    pressures_kpa: dict          # ab_id -> float
    piston_lengths_mm: dict      # piston_id -> float
    piston_homed: dict           # piston_id -> bool
    ab_modes: dict                # ab_id -> 'open_loop' | 'pid'
    ab_sensor_connected: dict    # ab_id -> bool, detected at boot


def parse_telemetry(line: str):
    """Parse a
    `TEL <t_ms> <p_prox> <p_cent> <p_dist> <p1>..<d3> <p1_homed>..<d3_homed>
         <prox_mode> <cent_mode> <dist_mode> <prox_sensor_ok> <cent_sensor_ok> <dist_sensor_ok>`
    line.

    Returns a Telemetry instance, or None if the line is not a well-formed TEL line.
    """
    fields = line.split()
    # "TEL" + t_ms + pressures + piston lengths + piston homed flags + ab modes + ab sensor flags
    expected = (
        1 + 1 + len(AB_IDS) + len(PISTON_IDS) + len(PISTON_IDS) + len(AB_IDS) + len(AB_IDS)
    )
    if len(fields) != expected or fields[0] != 'TEL':
        return None

    idx = 1
    try:
        t_ms = int(fields[idx]); idx += 1
        pressure_values = [float(v) for v in fields[idx:idx + len(AB_IDS)]]
        idx += len(AB_IDS)
        piston_values = [float(v) for v in fields[idx:idx + len(PISTON_IDS)]]
        idx += len(PISTON_IDS)
        homed_values = [v == '1' for v in fields[idx:idx + len(PISTON_IDS)]]
        idx += len(PISTON_IDS)
        mode_tokens = fields[idx:idx + len(AB_IDS)]
        idx += len(AB_IDS)
        ab_modes = [_AB_MODE_WIRE_TO_NAME.get(tok) for tok in mode_tokens]
        if any(mode is None for mode in ab_modes):
            return None
        sensor_values = [v == '1' for v in fields[idx:idx + len(AB_IDS)]]
    except ValueError:
        return None

    return Telemetry(
        t_ms=t_ms,
        pressures_kpa=dict(zip(AB_IDS, pressure_values)),
        piston_lengths_mm=dict(zip(PISTON_IDS, piston_values)),
        piston_homed=dict(zip(PISTON_IDS, homed_values)),
        ab_modes=dict(zip(AB_IDS, ab_modes)),
        ab_sensor_connected=dict(zip(AB_IDS, sensor_values)),
    )


def parse_ack(line: str):
    """Return the echoed command text from an `ACK <command_echo>` line, or None."""
    if not line.startswith('ACK'):
        return None
    rest = line[len('ACK'):].strip()
    return rest


def parse_err(line: str):
    """Return the message from an `ERR <message>` line, or None."""
    if not line.startswith('ERR'):
        return None
    rest = line[len('ERR'):].strip()
    return rest
