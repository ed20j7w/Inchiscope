from inchiscope_serial_bridge import protocol


def test_format_piston_command():
    assert protocol.format_piston_command('d1', 45.0) == 'PISTON d1 45.000'


def test_format_piston_command_rejects_unknown_id():
    try:
        protocol.format_piston_command('x9', 1.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_home_command():
    assert protocol.format_home_command('d1') == 'HOME d1'
    assert protocol.format_home_command('ALL') == 'HOME ALL'


def test_format_home_command_rejects_unknown_target():
    try:
        protocol.format_home_command('x9')
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_piston_range_command():
    assert protocol.format_piston_range_command('d1', 0.0, 100.0) == 'PISTON_RANGE d1 0.000 100.000'
    assert protocol.format_piston_range_command('ALL', 0.0, 100.0) == 'PISTON_RANGE ALL 0.000 100.000'


def test_format_piston_range_command_rejects_unknown_target():
    try:
        protocol.format_piston_range_command('x9', 0.0, 100.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_piston_range_command_rejects_inverted_range():
    try:
        protocol.format_piston_range_command('d1', 100.0, 0.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_piston_speed_command():
    assert protocol.format_piston_speed_command('d1', 60.0) == 'PISTON_SPEED d1 60.000'
    assert protocol.format_piston_speed_command('ALL', 60.0) == 'PISTON_SPEED ALL 60.000'


def test_format_piston_speed_command_rejects_unknown_target():
    try:
        protocol.format_piston_speed_command('x9', 60.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_piston_speed_command_rejects_non_positive_speed():
    try:
        protocol.format_piston_speed_command('d1', 0.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_valve_command_clamps_signed_duty():
    assert protocol.format_valve_command('central', 150) == 'VALVE central 100'
    assert protocol.format_valve_command('central', -150) == 'VALVE central -100'
    assert protocol.format_valve_command('central', -10) == 'VALVE central -10'


def test_format_ab_pid_command():
    assert protocol.format_ab_pid_command('distal', 42.5) == 'AB_PID distal 42.50'


def test_format_regulator_command():
    assert protocol.format_regulator_command('positive', 80.0) == 'REG POS 80.00'
    assert protocol.format_regulator_command('negative', -60.0) == 'REG NEG -60.00'


def test_format_regulator_command_rejects_unknown_id():
    try:
        protocol.format_regulator_command('sideways', 1.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_format_regulator_range_command():
    assert (
        protocol.format_regulator_range_command('positive', 0.0, 100.0)
        == 'REG_RANGE POS 0.00 100.00'
    )
    assert (
        protocol.format_regulator_range_command('negative', -100.0, 0.0)
        == 'REG_RANGE NEG -100.00 0.00'
    )


def test_format_regulator_range_command_rejects_inverted_range():
    try:
        protocol.format_regulator_range_command('positive', 100.0, 0.0)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_parse_telemetry_round_trip():
    line = (
        'TEL 12345 10.0 20.0 30.0 1.0 2.0 3.0 4.0 5.0 6.0 '
        '0 0 0 1 1 1 O P O 1 0 1'
    )
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
    assert telemetry.piston_homed == {
        'p1': False,
        'p2': False,
        'p3': False,
        'd1': True,
        'd2': True,
        'd3': True,
    }
    assert telemetry.ab_modes == {
        'proximal': 'open_loop',
        'central': 'pid',
        'distal': 'open_loop',
    }
    assert telemetry.ab_sensor_connected == {
        'proximal': True,
        'central': False,
        'distal': True,
    }


def test_parse_telemetry_rejects_malformed_line():
    assert protocol.parse_telemetry('TEL 1 2 3') is None
    assert protocol.parse_telemetry('ACK PING') is None


def test_parse_telemetry_rejects_bad_mode_token():
    line = (
        'TEL 12345 10.0 20.0 30.0 1.0 2.0 3.0 4.0 5.0 6.0 '
        '0 0 0 1 1 1 X P O 1 0 1'
    )
    assert protocol.parse_telemetry(line) is None


def test_parse_ack_and_err():
    assert protocol.parse_ack('ACK PING') == 'PING'
    assert protocol.parse_err('ERR unknown command: FOO') == 'unknown command: FOO'
    assert protocol.parse_ack('ERR nope') is None
