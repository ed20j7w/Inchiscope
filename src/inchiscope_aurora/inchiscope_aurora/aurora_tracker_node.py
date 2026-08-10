import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

SENSOR_INDEX = 0  # only one 6DOF sensor for now; sensor_1/sensor_2 come later (one per AB)


class AuroraTrackerNode(Node):
    """Publishes EM tracker pose for sensor_0 and broadcasts the matching tf2 transform.

    OPEN ITEM (inchiscope_ros2_architecture.md section 2, flagged there as a
    spike rather than an assumption): the vendor SDK ("Combined API Sample
    C++ v1.9.7") is a C++ class API (CombinedApi.h/.cpp) with a prebuilt
    libndicapi.so for Linux -- there are no official Python bindings, and it
    is NDI's proprietary sample code (all rights reserved, no redistribution
    grant in its license.txt), so it is NOT vendored into this repo. Expect
    it to be dropped locally under `third_party/ndi_combined_api/` (see
    .gitignore) before building the real integration. Someone still needs
    to pick one of:
      (a) a pybind11 extension wrapping CombinedApi, imported from here, or
      (b) a small standalone C++ ROS2 node instead (this package would then
          need to become ament_cmake rather than ament_python).
    Either way: generate/write the virtual SROM at startup, open the field
    generator connection, and poll pose per the vendor sample's control flow
    (sample/src/main.cpp in the vendor zip).

    Until that binding exists, this node only wires up the ROS2-side
    interface (pose topic, tf2 broadcaster, SROM path + port params) so
    downstream nodes and launch files have something to point at.
    """

    def __init__(self):
        super().__init__('aurora_tracker_node')

        self.declare_parameter('srom_path', '')
        self.declare_parameter('field_generator_port', '/dev/ttyUSB0')
        self.declare_parameter('publish_rate_hz', 40.0)

        self._pose_pub = self.create_publisher(
            PoseStamped, f'/aurora/sensor_{SENSOR_INDEX}/pose', 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate_hz, self._poll_and_publish)

    def _poll_and_publish(self):
        self.get_logger().warn(
            'Aurora SDK binding not yet implemented -- see AuroraTrackerNode docstring',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = AuroraTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
