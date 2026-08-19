"""Pure float<->int mapping between a QSlider (int-only) and a
QDoubleSpinBox (float), kept free of any Qt import so it's unit-testable
without PyQt installed.
"""

SLIDER_STEPS = 1000


def value_to_slider(value: float, minimum: float, maximum: float, steps: int = SLIDER_STEPS) -> int:
    """Maps value in [minimum, maximum] to a slider position in [0, steps].

    Clamps out-of-range values rather than raising, since callers pass live
    spinbox contents that may transiently be out of range while being typed.
    """
    if maximum <= minimum:
        return 0
    fraction = (value - minimum) / (maximum - minimum)
    fraction = min(1.0, max(0.0, fraction))
    return round(fraction * steps)


def slider_to_value(slider_pos: int, minimum: float, maximum: float, steps: int = SLIDER_STEPS) -> float:
    """Inverse of value_to_slider."""
    if steps <= 0:
        return minimum
    fraction = slider_pos / steps
    fraction = min(1.0, max(0.0, fraction))
    return minimum + fraction * (maximum - minimum)
