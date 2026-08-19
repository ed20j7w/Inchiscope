"""Owns the rclpy node and pumps it from the Qt event loop.

rclpy.spin_once() is called from a QTimer on the Qt main thread instead of a
background thread, so subscription callbacks run on the same thread as the
Qt event loop and can safely emit Qt signals directly -- no cross-thread
signal/slot plumbing needed, unlike serial_bridge_node's reader-thread
design (that one has to cross a real thread boundary; this one doesn't).
"""
import time

import rclpy
from rclpy.node import Node
from PyQt5 import QtCore

from inchiscope_msgs.msg import (
    AbPidCommand,
    HomeCommand,
    PistonCommand,
    PistonRangeCommand,
    PistonSpeedCommand,
    PistonStateArray,
    PressureStateArray,
    RegulatorCommand,
    RegulatorRangeCommand,
    ValveCommand,
)

SPIN_PERIOD_SEC = 0.02  # 50Hz -- matches the bridge's own telemetry drain rate


class _DebugGuiNode(Node):

    def __init__(self, on_piston_state, on_pressure_state):
        super().__init__('inchiscope_debug_gui')

        self.create_subscription(
            PistonStateArray, '/firmware/piston_state',
            lambda msg: on_piston_state({p.id: p for p in msg.pistons}), 10,
        )
        self.create_subscription(
            PressureStateArray, '/firmware/pressure_state',
            lambda msg: on_pressure_state({p.ab_id: p for p in msg.pressures}), 10,
        )

        self.piston_cmd_pub = self.create_publisher(PistonCommand, '/firmware/piston_cmd', 10)
        self.home_cmd_pub = self.create_publisher(HomeCommand, '/firmware/home_cmd', 10)
        self.piston_range_cmd_pub = self.create_publisher(
            PistonRangeCommand, '/firmware/piston_range_cmd', 10
        )
        self.piston_speed_cmd_pub = self.create_publisher(
            PistonSpeedCommand, '/firmware/piston_speed_cmd', 10
        )
        self.valve_cmd_pub = self.create_publisher(ValveCommand, '/firmware/valve_cmd', 10)
        self.ab_pid_cmd_pub = self.create_publisher(AbPidCommand, '/firmware/ab_pid_cmd', 10)
        self.regulator_cmd_pub = self.create_publisher(RegulatorCommand, '/firmware/regulator_cmd', 10)
        self.regulator_range_cmd_pub = self.create_publisher(
            RegulatorRangeCommand, '/firmware/regulator_range_cmd', 10
        )


class RosBridge(QtCore.QObject):
    """Qt-facing wrapper: emits telemetry as signals, exposes one publish
    method per command topic the bridge node subscribes to."""

    pistonStateReceived = QtCore.pyqtSignal(dict)
    pressureStateReceived = QtCore.pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        rclpy.init(args=None)
        self._node = _DebugGuiNode(
            on_piston_state=self.pistonStateReceived.emit,
            on_pressure_state=self.pressureStateReceived.emit,
        )
        self._last_telemetry_time = None
        self.pistonStateReceived.connect(self._mark_telemetry_seen)
        self.pressureStateReceived.connect(self._mark_telemetry_seen)

        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(int(SPIN_PERIOD_SEC * 1000))

    def _mark_telemetry_seen(self, _msg):
        self._last_telemetry_time = time.monotonic()

    def seconds_since_telemetry(self):
        """None if no telemetry has ever been received."""
        if self._last_telemetry_time is None:
            return None
        return time.monotonic() - self._last_telemetry_time

    def _spin_once(self):
        rclpy.spin_once(self._node, timeout_sec=0)

    def shutdown(self):
        self._spin_timer.stop()
        self._node.destroy_node()
        rclpy.shutdown()

    # -- outgoing commands, one per /firmware/*_cmd topic --------------------

    def send_piston_cmd(self, piston_id: str, target_length_mm: float):
        msg = PistonCommand()
        msg.id = piston_id
        msg.target_length_mm = float(target_length_mm)
        self._node.piston_cmd_pub.publish(msg)

    def send_home_cmd(self, piston_id: str):
        msg = HomeCommand()
        msg.id = piston_id
        self._node.home_cmd_pub.publish(msg)

    def send_piston_range_cmd(self, piston_id: str, min_mm: float, max_mm: float):
        msg = PistonRangeCommand()
        msg.id = piston_id
        msg.min_mm = float(min_mm)
        msg.max_mm = float(max_mm)
        self._node.piston_range_cmd_pub.publish(msg)

    def send_piston_speed_cmd(self, piston_id: str, mm_per_s: float):
        msg = PistonSpeedCommand()
        msg.id = piston_id
        msg.mm_per_s = float(mm_per_s)
        self._node.piston_speed_cmd_pub.publish(msg)

    def send_valve_cmd(self, ab_id: str, duty_pct: float):
        msg = ValveCommand()
        msg.ab_id = ab_id
        msg.duty_pct = float(duty_pct)
        self._node.valve_cmd_pub.publish(msg)

    def send_ab_pid_cmd(self, ab_id: str, target_kpa: float):
        msg = AbPidCommand()
        msg.ab_id = ab_id
        msg.target_kpa = float(target_kpa)
        self._node.ab_pid_cmd_pub.publish(msg)

    def send_regulator_cmd(self, regulator_id: str, target_kpa: float):
        msg = RegulatorCommand()
        msg.regulator_id = regulator_id
        msg.target_kpa = float(target_kpa)
        self._node.regulator_cmd_pub.publish(msg)

    def send_regulator_range_cmd(self, regulator_id: str, min_kpa: float, max_kpa: float):
        msg = RegulatorRangeCommand()
        msg.regulator_id = regulator_id
        msg.min_kpa = float(min_kpa)
        msg.max_kpa = float(max_kpa)
        self._node.regulator_range_cmd_pub.publish(msg)
