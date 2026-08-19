from PyQt5 import QtWidgets

from .. import constants
from ..widgets import SliderSpinBox, StatusDot


class _PistonCard(QtWidgets.QGroupBox):
    """One piston's readback + target slider + Home + range/speed config."""

    def __init__(self, piston_id, connected, parent=None):
        super().__init__(piston_id, parent)
        self.piston_id = piston_id
        self.connected = connected
        self.synced_from_telemetry = False

        layout = QtWidgets.QVBoxLayout(self)

        readback_row = QtWidgets.QHBoxLayout()
        self.length_label = QtWidgets.QLabel('length: -- mm')
        self.homed_dot = StatusDot('grey')
        self.homed_label = QtWidgets.QLabel('homed: --')
        self.at_target_dot = StatusDot('grey')
        self.at_target_label = QtWidgets.QLabel('at target: --')
        for w in (self.length_label, self.homed_dot, self.homed_label,
                  self.at_target_dot, self.at_target_label):
            readback_row.addWidget(w)
        readback_row.addStretch(1)
        layout.addLayout(readback_row)

        target_row = QtWidgets.QHBoxLayout()
        target_row.addWidget(QtWidgets.QLabel('Target length:'))
        self.target_slider = SliderSpinBox(
            constants.DEFAULT_PISTON_MIN_MM, constants.DEFAULT_PISTON_MAX_MM,
            decimals=1, suffix=' mm',
        )
        target_row.addWidget(self.target_slider, 1)
        layout.addLayout(target_row)

        config_row = QtWidgets.QHBoxLayout()
        self.home_button = QtWidgets.QPushButton('Home')
        config_row.addWidget(self.home_button)

        config_row.addWidget(QtWidgets.QLabel('Range:'))
        self.min_spin = QtWidgets.QDoubleSpinBox()
        self.min_spin.setRange(-1000.0, 1000.0)
        self.min_spin.setValue(constants.DEFAULT_PISTON_MIN_MM)
        self.min_spin.setSuffix(' mm')
        self.max_spin = QtWidgets.QDoubleSpinBox()
        self.max_spin.setRange(-1000.0, 1000.0)
        self.max_spin.setValue(constants.DEFAULT_PISTON_MAX_MM)
        self.max_spin.setSuffix(' mm')
        self.apply_range_button = QtWidgets.QPushButton('Apply Range')
        config_row.addWidget(self.min_spin)
        config_row.addWidget(QtWidgets.QLabel('to'))
        config_row.addWidget(self.max_spin)
        config_row.addWidget(self.apply_range_button)

        config_row.addWidget(QtWidgets.QLabel('Speed:'))
        self.speed_spin = QtWidgets.QDoubleSpinBox()
        self.speed_spin.setRange(constants.MIN_PISTON_SPEED_MM_S, constants.MAX_PISTON_SPEED_MM_S)
        self.speed_spin.setValue(constants.DEFAULT_PISTON_SPEED_MM_S)
        self.speed_spin.setSuffix(' mm/s')
        self.apply_speed_button = QtWidgets.QPushButton('Apply Speed')
        config_row.addWidget(self.speed_spin)
        config_row.addWidget(self.apply_speed_button)
        layout.addLayout(config_row)

        if not connected:
            note = QtWidgets.QLabel('Not wired on this bench setup (reserved for a second segment).')
            note.setStyleSheet('color: #6b7785; font-style: italic;')
            layout.addWidget(note)
            self.setEnabled(False)
        else:
            self.target_slider.set_enabled_control(False)  # stays off until homed=true

    def update_state(self, state):
        self.length_label.setText(f'length: {state.length_mm:.2f} mm')
        self.homed_label.setText(f'homed: {"yes" if state.homed else "no"}')
        self.homed_dot.set_color('green' if state.homed else 'amber')
        self.at_target_label.setText(f'at target: {"yes" if state.at_target else "no"}')
        self.at_target_dot.set_color('green' if state.at_target else 'grey')

        if not self.synced_from_telemetry:
            self.target_slider.set_value_programmatic(state.target_length_mm)
            self.synced_from_telemetry = True

        self.target_slider.set_enabled_control(state.homed)


class ActuatorsPanel(QtWidgets.QGroupBox):
    """The 6 pistons on /firmware/piston_cmd, /firmware/home_cmd,
    /firmware/piston_range_cmd, /firmware/piston_speed_cmd.

    A piston's target slider is synced to its live target_length_mm from
    telemetry exactly once (on first message) so arming doesn't yank it to
    an unrelated value, then only the heartbeat (while armed and touched)
    or direct user input moves it after that. It stays disabled until the
    piston reports homed=true, mirroring firmware's own PISTON-before-HOME
    rejection instead of just letting every send silently error out.
    """

    def __init__(self, ros_bridge, is_armed_fn, status_cb, parent=None):
        super().__init__('Actuators', parent)
        self._ros_bridge = ros_bridge
        self._is_armed = is_armed_fn
        self._status = status_cb
        self._touched = {pid: False for pid in constants.PISTON_IDS}
        self._cards = {}

        layout = QtWidgets.QVBoxLayout(self)

        home_all_row = QtWidgets.QHBoxLayout()
        home_all_button = QtWidgets.QPushButton('Home All')
        home_all_button.clicked.connect(self._on_home_all_clicked)
        home_all_row.addWidget(home_all_button)
        home_all_row.addStretch(1)
        layout.addLayout(home_all_row)

        distal_group = QtWidgets.QGroupBox('Distal segment (d1-d3) -- connected')
        distal_layout = QtWidgets.QVBoxLayout(distal_group)
        proximal_group = QtWidgets.QGroupBox('Proximal segment (p1-p3) -- reserved, not wired')
        proximal_layout = QtWidgets.QVBoxLayout(proximal_group)

        for piston_id in constants.PISTON_IDS:
            connected = constants.PISTON_CONNECTED[piston_id]
            card = _PistonCard(piston_id, connected)
            card.target_slider.valueChangedByUser.connect(
                lambda _v, pid=piston_id: self._on_target_touched(pid)
            )
            card.home_button.clicked.connect(
                lambda _c, pid=piston_id: self._on_home_clicked(pid)
            )
            card.apply_range_button.clicked.connect(
                lambda _c, pid=piston_id: self._on_apply_range(pid)
            )
            card.apply_speed_button.clicked.connect(
                lambda _c, pid=piston_id: self._on_apply_speed(pid)
            )
            self._cards[piston_id] = card
            (distal_layout if connected else proximal_layout).addWidget(card)

        layout.addWidget(distal_group)
        layout.addWidget(proximal_group)

    def _on_target_touched(self, piston_id):
        self._touched[piston_id] = True

    def _on_home_clicked(self, piston_id):
        if not self._is_armed():
            self._status('Arm the interface before homing.')
            return
        reply = QtWidgets.QMessageBox.question(
            self, 'Confirm homing',
            f'Home "{piston_id}"? This blind-retracts it toward its mechanical '
            'minimum for a fixed duration computed from its configured range/speed.',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._ros_bridge.send_home_cmd(piston_id)
        self._cards[piston_id].synced_from_telemetry = False
        self._status(f'Sent HOME for {piston_id}')

    def _on_home_all_clicked(self):
        if not self._is_armed():
            self._status('Arm the interface before homing.')
            return
        reply = QtWidgets.QMessageBox.question(
            self, 'Confirm homing all',
            'Home ALL connected pistons? Each blind-retracts toward its '
            'mechanical minimum simultaneously.',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return
        self._ros_bridge.send_home_cmd('ALL')
        for card in self._cards.values():
            card.synced_from_telemetry = False
        self._status('Sent HOME for ALL')

    def _on_apply_range(self, piston_id):
        if not self._is_armed():
            self._status('Arm the interface before applying a piston range.')
            return
        card = self._cards[piston_id]
        min_mm, max_mm = card.min_spin.value(), card.max_spin.value()
        self._ros_bridge.send_piston_range_cmd(piston_id, min_mm, max_mm)
        card.target_slider.set_range(min_mm, max_mm)
        self._status(f'Applied {piston_id} range: {min_mm:.1f} to {max_mm:.1f} mm (takes effect at next HOME)')

    def _on_apply_speed(self, piston_id):
        if not self._is_armed():
            self._status('Arm the interface before applying a piston speed.')
            return
        card = self._cards[piston_id]
        mm_per_s = card.speed_spin.value()
        self._ros_bridge.send_piston_speed_cmd(piston_id, mm_per_s)
        self._status(f'Applied {piston_id} speed: {mm_per_s:.1f} mm/s (takes effect at next HOME)')

    def update_piston_state(self, piston_by_id):
        for piston_id, state in piston_by_id.items():
            card = self._cards.get(piston_id)
            if card is not None:
                card.update_state(state)

    def heartbeat_tick(self):
        if not self._is_armed():
            return
        for piston_id, touched in self._touched.items():
            if not touched:
                continue
            card = self._cards[piston_id]
            if not card.target_slider.is_enabled_control():
                continue
            self._ros_bridge.send_piston_cmd(piston_id, card.target_slider.value())
