from PyQt5 import QtWidgets

from .. import constants
from ..widgets import SliderSpinBox


class RegulatorsPanel(QtWidgets.QGroupBox):
    """Regulators + duty cycles: the two shared analog regulators, the three
    AB open-loop valves, and the experimental AB_PID hold -- everything on
    /firmware/regulator_cmd, /firmware/regulator_range_cmd,
    /firmware/valve_cmd, and /firmware/ab_pid_cmd.

    Regulator and valve sliders are "touched" (armed heartbeat picks them
    up) only once the user actually moves them -- there's no readback for
    either topic, so streaming a default-0 value the instant you hit ARM
    could silently stomp on a setpoint someone already dialled in by hand.
    AB_PID has no heartbeat at all: it's a one-shot "Set" button behind an
    explicit acknowledgement, since the firmware README flags its gains as
    untested placeholders.
    """

    def __init__(self, ros_bridge, is_armed_fn, status_cb, parent=None):
        super().__init__('Regulators && Duty Cycles', parent)
        self._ros_bridge = ros_bridge
        self._is_armed = is_armed_fn
        self._status = status_cb

        self._regulator_sliders = {}
        self._regulator_touched = {rid: False for rid in constants.REGULATOR_IDS}
        self._regulator_range_spins = {}

        self._valve_sliders = {}
        self._valve_touched = {ab_id: False for ab_id in constants.AB_IDS}
        self._valve_mode_labels = {}

        self._ab_pid_sliders = {}
        self._ab_pid_set_buttons = {}

        outer = QtWidgets.QVBoxLayout(self)
        outer.addWidget(self._build_regulators_group())
        outer.addWidget(self._build_valves_group())
        outer.addWidget(self._build_ab_pid_group())

    # -- Pressure regulators --------------------------------------------------

    def _build_regulators_group(self):
        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel('<b>Regulator</b>'), 0, 0)
        grid.addWidget(QtWidgets.QLabel('<b>Target pressure (kPa)</b>'), 0, 1)
        grid.addWidget(QtWidgets.QLabel('<b>Configured range (kPa)</b>'), 0, 2)

        for row, reg_id in enumerate(constants.REGULATOR_IDS, start=1):
            if reg_id == 'positive':
                minimum, maximum = 0.0, constants.DEFAULT_REG_POS_MAX_KPA
            else:
                minimum, maximum = constants.DEFAULT_REG_NEG_MIN_KPA, 0.0

            grid.addWidget(QtWidgets.QLabel(reg_id), row, 0)

            slider = SliderSpinBox(minimum, maximum, decimals=1, suffix=' kPa', initial=0.0)
            slider.valueChangedByUser.connect(
                lambda _v, rid=reg_id: self._on_regulator_touched(rid)
            )
            self._regulator_sliders[reg_id] = slider
            grid.addWidget(slider, row, 1)

            range_row = QtWidgets.QHBoxLayout()
            min_spin = QtWidgets.QDoubleSpinBox()
            min_spin.setRange(-1000.0, 1000.0)
            min_spin.setValue(minimum)
            min_spin.setSuffix(' kPa')
            max_spin = QtWidgets.QDoubleSpinBox()
            max_spin.setRange(-1000.0, 1000.0)
            max_spin.setValue(maximum)
            max_spin.setSuffix(' kPa')
            apply_btn = QtWidgets.QPushButton('Apply Range')
            apply_btn.clicked.connect(
                lambda _c, rid=reg_id: self._apply_regulator_range(rid)
            )
            self._regulator_range_spins[reg_id] = (min_spin, max_spin)
            range_row.addWidget(min_spin)
            range_row.addWidget(QtWidgets.QLabel('to'))
            range_row.addWidget(max_spin)
            range_row.addWidget(apply_btn)
            range_widget = QtWidgets.QWidget()
            range_widget.setLayout(range_row)
            grid.addWidget(range_widget, row, 2)

        note = QtWidgets.QLabel(
            'Open-loop: the Mega outputs a DAC voltage that IS the setpoint -- '
            'there is no feedback sensor on these lines, so this GUI has no way '
            'to read back the regulator\'s actual current output.'
        )
        note.setWordWrap(True)
        note.setStyleSheet('color: #6b7785; font-style: italic;')
        container = QtWidgets.QGroupBox('Pressure Regulators (shared across all ABs)')
        outer_layout = QtWidgets.QVBoxLayout(container)
        outer_layout.addLayout(grid)
        outer_layout.addWidget(note)
        return container

    def _on_regulator_touched(self, regulator_id):
        self._regulator_touched[regulator_id] = True

    def _apply_regulator_range(self, regulator_id):
        if not self._is_armed():
            self._status('Arm the interface before applying a regulator range.')
            return
        min_spin, max_spin = self._regulator_range_spins[regulator_id]
        min_kpa, max_kpa = min_spin.value(), max_spin.value()
        self._ros_bridge.send_regulator_range_cmd(regulator_id, min_kpa, max_kpa)
        self._regulator_sliders[regulator_id].set_range(min_kpa, max_kpa)
        self._status(f'Applied {regulator_id} regulator range: {min_kpa:.1f} to {max_kpa:.1f} kPa')

    # -- AB valves (open-loop duty) --------------------------------------------

    def _build_valves_group(self):
        group = QtWidgets.QGroupBox('AB Valves -- open-loop duty cycle')
        grid = QtWidgets.QGridLayout(group)
        grid.addWidget(QtWidgets.QLabel('<b>AB</b>'), 0, 0)
        grid.addWidget(QtWidgets.QLabel('<b>Duty (-100=vacuum .. +100=inflate)</b>'), 0, 1)
        grid.addWidget(QtWidgets.QLabel('<b>Mode</b>'), 0, 2)

        for row, ab_id in enumerate(constants.AB_IDS, start=1):
            grid.addWidget(QtWidgets.QLabel(ab_id), row, 0)

            slider = SliderSpinBox(
                constants.VALVE_DUTY_MIN_PCT, constants.VALVE_DUTY_MAX_PCT,
                decimals=1, suffix=' %', initial=0.0,
            )
            slider.valueChangedByUser.connect(
                lambda _v, aid=ab_id: self._on_valve_touched(aid)
            )
            self._valve_sliders[ab_id] = slider
            grid.addWidget(slider, row, 1)

            mode_label = QtWidgets.QLabel('unknown')
            self._valve_mode_labels[ab_id] = mode_label
            grid.addWidget(mode_label, row, 2)

        note = QtWidgets.QLabel(
            'Sending a duty command always forces that AB into open-loop mode, '
            'overriding any active AB_PID hold.'
        )
        note.setWordWrap(True)
        note.setStyleSheet('color: #6b7785; font-style: italic;')
        layout = group.layout()
        layout.addWidget(note, len(constants.AB_IDS) + 1, 0, 1, 3)
        return group

    def _on_valve_touched(self, ab_id):
        self._valve_touched[ab_id] = True

    # -- AB_PID (experimental) -------------------------------------------------

    def _build_ab_pid_group(self):
        group = QtWidgets.QGroupBox('AB_PID -- firmware-side pressure hold (EXPERIMENTAL)')
        layout = QtWidgets.QVBoxLayout(group)

        warning = QtWidgets.QLabel(
            '<b>Untested.</b> Placeholder gains (Kp=2.0, Ki=0.5, Kd=0.05), sign '
            'convention unverified against real hardware -- see '
            'inchiscope_serial_bridge/README.md. May cause unexpected valve motion.'
        )
        warning.setWordWrap(True)
        warning.setStyleSheet('color: #a05a0e;')
        layout.addWidget(warning)

        self._ab_pid_enable_checkbox = QtWidgets.QCheckBox(
            'I understand this is untested and want to enable these controls'
        )
        self._ab_pid_enable_checkbox.toggled.connect(self._on_ab_pid_enable_toggled)
        layout.addWidget(self._ab_pid_enable_checkbox)

        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel('<b>AB</b>'), 0, 0)
        grid.addWidget(QtWidgets.QLabel('<b>Target pressure (kPa)</b>'), 0, 1)
        grid.addWidget(QtWidgets.QLabel(''), 0, 2)

        for row, ab_id in enumerate(constants.AB_IDS, start=1):
            grid.addWidget(QtWidgets.QLabel(ab_id), row, 0)
            slider = SliderSpinBox(-100.0, 100.0, decimals=1, suffix=' kPa', initial=0.0)
            slider.set_enabled_control(False)
            self._ab_pid_sliders[ab_id] = slider
            grid.addWidget(slider, row, 1)

            set_btn = QtWidgets.QPushButton('Set PID Target')
            set_btn.setEnabled(False)
            set_btn.clicked.connect(lambda _c, aid=ab_id: self._send_ab_pid(aid))
            self._ab_pid_set_buttons[ab_id] = set_btn
            grid.addWidget(set_btn, row, 2)

        layout.addLayout(grid)
        return group

    def _on_ab_pid_enable_toggled(self, checked):
        for ab_id in constants.AB_IDS:
            self._ab_pid_sliders[ab_id].set_enabled_control(checked)
            self._ab_pid_set_buttons[ab_id].setEnabled(checked)

    def _send_ab_pid(self, ab_id):
        if not self._is_armed():
            self._status('Arm the interface before setting an AB_PID target.')
            return
        target = self._ab_pid_sliders[ab_id].value()
        self._ros_bridge.send_ab_pid_cmd(ab_id, target)
        self._status(f'Sent AB_PID target for {ab_id}: {target:.1f} kPa')

    # -- telemetry / heartbeat ---------------------------------------------

    def update_pressure_state(self, pressure_by_ab_id):
        for ab_id, state in pressure_by_ab_id.items():
            mode_label = self._valve_mode_labels.get(ab_id)
            if mode_label is not None:
                mode_label.setText(state.mode)
            if not state.sensor_connected and ab_id in self._ab_pid_set_buttons:
                self._ab_pid_sliders[ab_id].set_enabled_control(False)
                self._ab_pid_set_buttons[ab_id].setEnabled(False)
                self._ab_pid_set_buttons[ab_id].setToolTip('Sensor not connected -- firmware will refuse AB_PID for this AB.')
            elif self._ab_pid_enable_checkbox.isChecked():
                self._ab_pid_sliders[ab_id].set_enabled_control(True)
                self._ab_pid_set_buttons[ab_id].setEnabled(True)
                self._ab_pid_set_buttons[ab_id].setToolTip('')

    def heartbeat_tick(self):
        if not self._is_armed():
            return
        for reg_id, touched in self._regulator_touched.items():
            if touched:
                self._ros_bridge.send_regulator_cmd(reg_id, self._regulator_sliders[reg_id].value())
        for ab_id, touched in self._valve_touched.items():
            if touched:
                self._ros_bridge.send_valve_cmd(ab_id, self._valve_sliders[ab_id].value())
