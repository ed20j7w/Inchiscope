from PyQt5 import QtCore, QtWidgets

from . import scaling


class SliderSpinBox(QtWidgets.QWidget):
    """A QSlider + QDoubleSpinBox kept in sync, representing one float value.

    valueChangedByUser fires only when the user drags the slider or edits
    the spinbox directly -- never when set_value_programmatic() is used to
    reflect a telemetry readback -- so callers can tell a live commanded
    change apart from a passive display update without extra bookkeeping.
    """

    valueChangedByUser = QtCore.pyqtSignal(float)

    def __init__(self, minimum, maximum, decimals=1, suffix='', initial=None, parent=None):
        super().__init__(parent)
        self._minimum = minimum
        self._maximum = maximum
        self._updating = False

        self._slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self._slider.setRange(0, scaling.SLIDER_STEPS)

        self._spin = QtWidgets.QDoubleSpinBox()
        self._spin.setDecimals(decimals)
        self._spin.setSuffix(suffix)
        self._spin.setRange(minimum, maximum)
        self._spin.setFixedWidth(90)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._slider, 1)
        layout.addWidget(self._spin)

        self._slider.valueChanged.connect(self._on_slider_changed)
        self._spin.valueChanged.connect(self._on_spin_changed)

        start = minimum if initial is None else min(max(initial, minimum), maximum)
        self.set_value_programmatic(start)

    def set_range(self, minimum, maximum):
        """Rescales to a new range, clamping the current value into it."""
        current = self.value()
        self._minimum = minimum
        self._maximum = maximum
        self._updating = True
        try:
            self._spin.setRange(minimum, maximum)
        finally:
            self._updating = False
        self.set_value_programmatic(min(max(current, minimum), maximum))

    def value(self) -> float:
        return self._spin.value()

    def set_value_programmatic(self, value: float):
        """Sets the displayed value without emitting valueChangedByUser."""
        self._updating = True
        try:
            self._spin.setValue(value)
            self._slider.setValue(scaling.value_to_slider(value, self._minimum, self._maximum))
        finally:
            self._updating = False

    def set_enabled_control(self, enabled: bool):
        self._slider.setEnabled(enabled)
        self._spin.setEnabled(enabled)

    def is_enabled_control(self) -> bool:
        return self._slider.isEnabled()

    def _on_slider_changed(self, pos):
        if self._updating:
            return
        value = scaling.slider_to_value(pos, self._minimum, self._maximum)
        self._updating = True
        try:
            self._spin.setValue(value)
        finally:
            self._updating = False
        self.valueChangedByUser.emit(value)

    def _on_spin_changed(self, value):
        if self._updating:
            return
        self._updating = True
        try:
            self._slider.setValue(scaling.value_to_slider(value, self._minimum, self._maximum))
        finally:
            self._updating = False
        self.valueChangedByUser.emit(value)


class StatusDot(QtWidgets.QLabel):
    """A small coloured circle used as a boolean/tri-state status indicator."""

    _COLORS = {
        'grey': '#9e9e9e',
        'green': '#2e7d32',
        'red': '#c62828',
        'amber': '#e2984b',
    }

    def __init__(self, color='grey', diameter=14, parent=None):
        super().__init__(parent)
        self.setFixedSize(diameter, diameter)
        self._diameter = diameter
        self.set_color(color)

    def set_color(self, color: str):
        hex_color = self._COLORS.get(color, self._COLORS['grey'])
        self.setStyleSheet(
            f'background-color: {hex_color}; border-radius: {self._diameter // 2}px;'
        )
