import queue
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from inchiscope_msgs.msg import (
    PistonCommand,
    PistonState,
    PistonStateArray,
    PressureState,
    PressureStateArray,
    ValveCommand,
)

from . import protocol


class SerialBridgeNode(Node):
    """Owns the serial link to the Mega and is a pure translation layer.

    No kinematics, no calibration here -- see inchiscope_ros2_architecture.md
    section 5. Reading happens on a background thread (serial reads block);
    everything that touches rclpy publishers runs back on the node's own
    timer so callbacks stay single-threaded from ROS2's point of view.
    """

    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('piston_at_target_threshold_mm', 0.1)
        self.declare_parameter('telemetry_timeout_sec', 1.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self._at_target_threshold = float(
            self.get_parameter('piston_at_target_threshold_mm').value
        )
        self._telemetry_timeout = float(
            self.get_parameter('telemetry_timeout_sec').value
        )

        self._piston_pub = self.create_publisher(
            PistonStateArray, '/firmware/piston_state', 10
        )
        self._pressure_pub = self.create_publisher(
            PressureStateArray, '/firmware/pressure_state', 10
        )

        self.create_subscription(
            PistonCommand, '/firmware/piston_cmd', self._on_piston_cmd, 10
        )
        self.create_subscription(
            ValveCommand, '/firmware/valve_cmd', self._on_valve_cmd, 10
        )

        self._piston_targets = {pid: None for pid in protocol.PISTON_IDS}
        self._last_telemetry_wall_time = None

        self._write_lock = threading.Lock()
        self._line_queue = queue.Queue()

        self._serial = serial.Serial(port, baud, timeout=0.2)
        self.get_logger().info(f'Opened serial port {port} @ {baud} baud')

        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # Firmware pushes TEL at 20 Hz; drain faster than that so ACK/ERR
        # lines don't pile up behind a telemetry line.
        self._drain_timer = self.create_timer(0.02, self._drain_queue)
        self._watchdog_timer = self.create_timer(0.5, self._check_watchdog)

    def destroy_node(self):
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._serial.is_open:
            self._serial.close()
        super().destroy_node()

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read failed: {exc}')
                time.sleep(0.5)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            if line:
                self._line_queue.put(line)

    def _drain_queue(self):
        while True:
            try:
                line = self._line_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)

    def _handle_line(self, line):
        telemetry = protocol.parse_telemetry(line)
        if telemetry is not None:
            self._publish_telemetry(telemetry)
            self._last_telemetry_wall_time = time.monotonic()
            return

        err = protocol.parse_err(line)
        if err is not None:
            self.get_logger().warn(f'Firmware ERR: {err}')
            return

        ack = protocol.parse_ack(line)
        if ack is not None:
            self.get_logger().debug(f'Firmware ACK: {ack}')
            return

        self.get_logger().debug(f'Unrecognised line from firmware: {line}')

    def _publish_telemetry(self, telemetry: protocol.Telemetry):
        piston_msg = PistonStateArray()
        for pid in protocol.PISTON_IDS:
            length_mm = telemetry.piston_lengths_mm[pid]
            target = self._piston_targets.get(pid)
            state = PistonState()
            state.id = pid
            state.length_mm = length_mm
            state.target_length_mm = target if target is not None else length_mm
            state.at_target = (
                target is not None
                and abs(length_mm - target) <= self._at_target_threshold
            )
            piston_msg.pistons.append(state)
        self._piston_pub.publish(piston_msg)

        pressure_msg = PressureStateArray()
        for ab_id in protocol.AB_IDS:
            state = PressureState()
            state.ab_id = ab_id
            state.pressure_kpa = telemetry.pressures_kpa[ab_id]
            # The bridge only ever commands a duty cycle, never a target
            # pressure, so it has no basis for target_pressure_kpa/at_target
            # -- that belongs to ab_control_node, which owns the
            # diameter->pressure mapping.
            state.target_pressure_kpa = 0.0
            state.at_target = False
            pressure_msg.pressures.append(state)
        self._pressure_pub.publish(pressure_msg)

    def _on_piston_cmd(self, msg: PistonCommand):
        try:
            line = protocol.format_piston_command(msg.id, msg.target_length_mm)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self._piston_targets[msg.id] = msg.target_length_mm
        self._write_line(line)

    def _on_valve_cmd(self, msg: ValveCommand):
        try:
            line = protocol.format_valve_command(msg.ab_id, msg.duty_pct)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self._write_line(line)

    def _write_line(self, line: str):
        with self._write_lock:
            try:
                self._serial.write((line + '\n').encode('ascii'))
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial write failed: {exc}')

    def _check_watchdog(self):
        if self._last_telemetry_wall_time is None:
            return
        elapsed = time.monotonic() - self._last_telemetry_wall_time
        if elapsed > self._telemetry_timeout:
            self.get_logger().warn(
                f'No telemetry from firmware for {elapsed:.2f}s '
                f'(timeout {self._telemetry_timeout:.2f}s)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
