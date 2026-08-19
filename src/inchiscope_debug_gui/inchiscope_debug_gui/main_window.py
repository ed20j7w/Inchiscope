from PyQt5 import QtCore, QtWidgets

from . import constants
from .panels.actuators_panel import ActuatorsPanel
from .panels.pressure_panel import PressureReadingsPanel
from .panels.regulators_panel import RegulatorsPanel
from .ros_bridge import RosBridge

WATCHDOG_PERIOD_SEC = 0.5


class MainWindow(QtWidgets.QMainWindow):
    """Debug GUI top level: an ARM bar, the three grouped panels, a
    heartbeat timer that streams touched live controls while armed, and a
    watchdog that auto-disarms if the bridge stops sending telemetry.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Inchiscope Debug GUI -- direct /firmware/* control')
        self.resize(1100, 900)

        self._armed = False
        self._ros_bridge = RosBridge()

        central = QtWidgets.QWidget()
        outer_layout = QtWidgets.QVBoxLayout(central)
        outer_layout.addWidget(self._build_arm_bar())

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_content)

        self._regulators_panel = RegulatorsPanel(self._ros_bridge, self.is_armed, self._show_status)
        self._actuators_panel = ActuatorsPanel(self._ros_bridge, self.is_armed, self._show_status)
        self._pressure_panel = PressureReadingsPanel()

        scroll_layout.addWidget(self._regulators_panel)
        scroll_layout.addWidget(self._actuators_panel)
        scroll_layout.addWidget(self._pressure_panel)
        scroll_layout.addStretch(1)
        scroll.setWidget(scroll_content)
        outer_layout.addWidget(scroll, 1)

        self.setCentralWidget(central)
        self.statusBar().showMessage('Not armed. All /firmware/*_cmd controls are inert.')

        self._ros_bridge.pistonStateReceived.connect(self._actuators_panel.update_piston_state)
        self._ros_bridge.pressureStateReceived.connect(self._pressure_panel.update_pressure_state)
        self._ros_bridge.pressureStateReceived.connect(self._regulators_panel.update_pressure_state)

        self._heartbeat_timer = QtCore.QTimer(self)
        self._heartbeat_timer.timeout.connect(self._on_heartbeat)
        self._heartbeat_timer.start(int(1000.0 / constants.HEARTBEAT_HZ))

        self._watchdog_timer = QtCore.QTimer(self)
        self._watchdog_timer.timeout.connect(self._on_watchdog)
        self._watchdog_timer.start(int(WATCHDOG_PERIOD_SEC * 1000))

    def is_armed(self) -> bool:
        return self._armed

    def _build_arm_bar(self):
        bar = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(bar)

        self._bridge_status_dot = QtWidgets.QLabel('●')
        self._bridge_status_dot.setStyleSheet('color: #9e9e9e; font-size: 16px;')
        self._bridge_status_label = QtWidgets.QLabel('Bridge: no data yet')
        layout.addWidget(self._bridge_status_dot)
        layout.addWidget(self._bridge_status_label)
        layout.addSpacing(24)

        self._arm_button = QtWidgets.QPushButton('ARM')
        self._arm_button.setCheckable(True)
        self._arm_button.setFixedWidth(140)
        self._arm_button.toggled.connect(self._on_arm_toggled)
        self._style_arm_button(False)
        layout.addWidget(self._arm_button)

        warning = QtWidgets.QLabel(
            'This bypasses the AB/PBA controllers and drives regulators, valves, '
            'and actuators directly. All commands are inert until armed.'
        )
        warning.setWordWrap(True)
        warning.setStyleSheet('color: #6b7785;')
        layout.addWidget(warning, 1)
        return bar

    def _style_arm_button(self, armed):
        if armed:
            self._arm_button.setText('ARMED')
            self._arm_button.setStyleSheet(
                'background-color: #2e7d32; color: white; font-weight: bold;'
            )
        else:
            self._arm_button.setText('DISARMED')
            self._arm_button.setStyleSheet(
                'background-color: #9e9e9e; color: white; font-weight: bold;'
            )

    def _on_arm_toggled(self, checked):
        self._armed = checked
        self._style_arm_button(checked)
        if checked:
            self._show_status('ARMED -- touched controls are now streaming at '
                               f'{constants.HEARTBEAT_HZ:.0f} Hz.')
        else:
            self._show_status('Disarmed. All /firmware/*_cmd controls are inert.')

    def _show_status(self, message):
        self.statusBar().showMessage(message, 5000)

    def _on_heartbeat(self):
        if not self._armed:
            return
        self._regulators_panel.heartbeat_tick()
        self._actuators_panel.heartbeat_tick()

    def _on_watchdog(self):
        elapsed = self._ros_bridge.seconds_since_telemetry()
        if elapsed is None:
            self._bridge_status_dot.setStyleSheet('color: #9e9e9e; font-size: 16px;')
            self._bridge_status_label.setText('Bridge: no data yet')
            return

        if elapsed > constants.TELEMETRY_TIMEOUT_SEC:
            self._bridge_status_dot.setStyleSheet('color: #c62828; font-size: 16px;')
            self._bridge_status_label.setText(f'Bridge: stale ({elapsed:.1f}s since last telemetry)')
            if self._armed:
                self._arm_button.setChecked(False)
                self._show_status(
                    f'Auto-disarmed: no telemetry for {elapsed:.1f}s '
                    f'(timeout {constants.TELEMETRY_TIMEOUT_SEC:.1f}s). Is serial_bridge_node running?'
                )
        else:
            self._bridge_status_dot.setStyleSheet('color: #2e7d32; font-size: 16px;')
            self._bridge_status_label.setText('Bridge: connected')

    def closeEvent(self, event):
        self._heartbeat_timer.stop()
        self._watchdog_timer.stop()
        self._ros_bridge.shutdown()
        super().closeEvent(event)
