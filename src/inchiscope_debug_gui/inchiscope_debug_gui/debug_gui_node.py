"""Entry point for `ros2 run inchiscope_debug_gui debug_gui`.

Runs a QApplication event loop directly (not rclpy.spin) -- the RosBridge
pumps rclpy from a QTimer on this same thread, see ros_bridge.py.
"""
import sys

from PyQt5 import QtWidgets

from .main_window import MainWindow


def main(args=None):
    app = QtWidgets.QApplication(sys.argv if args is None else args)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
