"""Pure encode/decode helpers for the PC<->Mega ASCII serial protocol.

Kept free of rclpy/pyserial so the wire format can be unit tested without
hardware or a ROS2 environment. See inchiscope_ros2_architecture.md section 3
for the protocol definition.
"""

from dataclasses import dataclass

PISTON_IDS = ('p1', 'p2', 'p3', 'd1', 'd2', 'd3')
AB_IDS = ('proximal', 'central', 'distal')


def format_piston_command(piston_id: str, target_length_mm: float) -> str:
    if piston_id not in PISTON_IDS:
        raise ValueError(f'unknown piston id: {piston_id}')
    return f'PISTON {piston_id} {target_length_mm:.3f}'


def format_valve_command(ab_id: str, duty_pct: float) -> str:
    if ab_id not in AB_IDS:
        raise ValueError(f'unknown ab id: {ab_id}')
    duty_pct = max(0, min(100, int(round(duty_pct))))
    return f'VALVE {ab_id} {duty_pct}'


def format_ping() -> str:
    return 'PING'


@dataclass
class Telemetry:
    t_ms: int
    pressures_kpa: dict  # ab_id -> float
    piston_lengths_mm: dict  # piston_id -> float


def parse_telemetry(line: str):
    """Parse a `TEL <t_ms> <p_prox> <p_cent> <p_dist> <p1> <p2> <p3> <d1> <d2> <d3>` line.

    Returns a Telemetry instance, or None if the line is not a well-formed TEL line.
    """
    fields = line.split()
    expected = 1 + 1 + len(AB_IDS) + len(PISTON_IDS)  # "TEL" + t_ms + pressures + pistons
    if len(fields) != expected or fields[0] != 'TEL':
        return None
    try:
        t_ms = int(fields[1])
        pressure_values = [float(v) for v in fields[2:2 + len(AB_IDS)]]
        piston_values = [float(v) for v in fields[2 + len(AB_IDS):]]
    except ValueError:
        return None

    return Telemetry(
        t_ms=t_ms,
        pressures_kpa=dict(zip(AB_IDS, pressure_values)),
        piston_lengths_mm=dict(zip(PISTON_IDS, piston_values)),
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
