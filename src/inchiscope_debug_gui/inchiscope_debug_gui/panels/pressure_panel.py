from PyQt5 import QtWidgets

from .. import constants
from ..widgets import StatusDot


class PressureReadingsPanel(QtWidgets.QGroupBox):
    """Read-only view of /firmware/pressure_state -- no commands originate
    here, just the live numbers for each AB."""

    def __init__(self, parent=None):
        super().__init__('Pressure Readings', parent)
        grid = QtWidgets.QGridLayout(self)

        headers = ['AB', 'Pressure', 'Sensor', 'Mode', 'PID target', 'At target']
        for col, text in enumerate(headers):
            grid.addWidget(QtWidgets.QLabel(f'<b>{text}</b>'), 0, col)

        self._pressure_labels = {}
        self._sensor_dots = {}
        self._mode_labels = {}
        self._target_labels = {}
        self._at_target_dots = {}

        for row, ab_id in enumerate(constants.AB_IDS, start=1):
            grid.addWidget(QtWidgets.QLabel(ab_id), row, 0)

            pressure_label = QtWidgets.QLabel('-- kPa')
            font = pressure_label.font()
            font.setPointSize(font.pointSize() + 3)
            font.setBold(True)
            pressure_label.setFont(font)
            self._pressure_labels[ab_id] = pressure_label
            grid.addWidget(pressure_label, row, 1)

            sensor_dot = StatusDot('grey')
            self._sensor_dots[ab_id] = sensor_dot
            grid.addWidget(sensor_dot, row, 2)

            mode_label = QtWidgets.QLabel('--')
            self._mode_labels[ab_id] = mode_label
            grid.addWidget(mode_label, row, 3)

            target_label = QtWidgets.QLabel('--')
            self._target_labels[ab_id] = target_label
            grid.addWidget(target_label, row, 4)

            at_target_dot = StatusDot('grey')
            self._at_target_dots[ab_id] = at_target_dot
            grid.addWidget(at_target_dot, row, 5)

    def update_pressure_state(self, pressure_by_ab_id):
        for ab_id, state in pressure_by_ab_id.items():
            if ab_id not in self._pressure_labels:
                continue
            self._pressure_labels[ab_id].setText(f'{state.pressure_kpa:.2f} kPa')
            self._sensor_dots[ab_id].set_color('green' if state.sensor_connected else 'red')
            self._mode_labels[ab_id].setText(state.mode)
            if state.mode == 'pid':
                self._target_labels[ab_id].setText(f'{state.target_pressure_kpa:.2f} kPa')
                self._at_target_dots[ab_id].set_color('green' if state.at_target else 'amber')
            else:
                self._target_labels[ab_id].setText('n/a (open loop)')
                self._at_target_dots[ab_id].set_color('grey')
