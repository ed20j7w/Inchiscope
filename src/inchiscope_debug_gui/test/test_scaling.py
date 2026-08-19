from inchiscope_debug_gui import scaling


def test_value_to_slider_endpoints():
    assert scaling.value_to_slider(0.0, 0.0, 100.0) == 0
    assert scaling.value_to_slider(100.0, 0.0, 100.0) == scaling.SLIDER_STEPS


def test_value_to_slider_midpoint():
    assert scaling.value_to_slider(50.0, 0.0, 100.0) == scaling.SLIDER_STEPS // 2


def test_value_to_slider_clamps_out_of_range():
    assert scaling.value_to_slider(-10.0, 0.0, 100.0) == 0
    assert scaling.value_to_slider(999.0, 0.0, 100.0) == scaling.SLIDER_STEPS


def test_value_to_slider_handles_negative_range():
    # Regulator negative line: -100..0 kPa.
    assert scaling.value_to_slider(-100.0, -100.0, 0.0) == 0
    assert scaling.value_to_slider(0.0, -100.0, 0.0) == scaling.SLIDER_STEPS
    assert scaling.value_to_slider(-50.0, -100.0, 0.0) == scaling.SLIDER_STEPS // 2


def test_value_to_slider_degenerate_range_returns_zero():
    assert scaling.value_to_slider(5.0, 10.0, 10.0) == 0


def test_slider_to_value_endpoints():
    assert scaling.slider_to_value(0, 0.0, 100.0) == 0.0
    assert scaling.slider_to_value(scaling.SLIDER_STEPS, 0.0, 100.0) == 100.0


def test_slider_to_value_clamps_out_of_range_positions():
    assert scaling.slider_to_value(-5, 0.0, 100.0) == 0.0
    assert scaling.slider_to_value(scaling.SLIDER_STEPS + 5, 0.0, 100.0) == 100.0


def test_round_trip_is_close_within_one_step():
    minimum, maximum = -100.0, 100.0
    for value in [-100.0, -37.5, 0.0, 12.3, 100.0]:
        pos = scaling.value_to_slider(value, minimum, maximum)
        recovered = scaling.slider_to_value(pos, minimum, maximum)
        step_size = (maximum - minimum) / scaling.SLIDER_STEPS
        assert abs(recovered - value) <= step_size
