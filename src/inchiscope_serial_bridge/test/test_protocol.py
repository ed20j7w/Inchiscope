from inchiscope_serial_bridge import protocol


def test_format_piston_command():
    assert protocol.format_piston_command('d1', 45.0) == 'PISTON d1 45.000'


def test_format_piston_command_rejects_unknown_id():
    try:
        protocol.format_piston_command('x9', 1.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_valve_command_clamps_duty():
    assert protocol.format_valve_command('central', 150) == 'VALVE central 100'
    assert protocol.format_valve_command('central', -10) == 'VALVE central 0'


def test_parse_telemetry_round_trip():
    line = 'TEL 12345 10.0 20.0 30.0 1.0 2.0 3.0 4.0 5.0 6.0'
    telemetry = protocol.parse_telemetry(line)
    assert telemetry is not None
    assert telemetry.t_ms == 12345
    assert telemetry.pressures_kpa == {
        'proximal': 10.0,
        'central': 20.0,
        'distal': 30.0,
    }
    assert telemetry.piston_lengths_mm == {
        'p1': 1.0,
        'p2': 2.0,
        'p3': 3.0,
        'd1': 4.0,
        'd2': 5.0,
        'd3': 6.0,
    }


def test_parse_telemetry_rejects_malformed_line():
    assert protocol.parse_telemetry('TEL 1 2 3') is None
    assert protocol.parse_telemetry('ACK PING') is None


def test_parse_ack_and_err():
    assert protocol.parse_ack('ACK PING') == 'PING'
    assert protocol.parse_err('ERR unknown command: FOO') == 'unknown command: FOO'
    assert protocol.parse_ack('ERR nope') is None
