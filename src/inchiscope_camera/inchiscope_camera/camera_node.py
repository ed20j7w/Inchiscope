import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    """Publishes the NanEye feed on /camera/image_raw.

    The NanEye sensor is fed through a USB capture card, which exposes it as
    a plain V4L2/UVC video device -- unlike Aurora, no vendor SDK is needed
    here, just OpenCV's VideoCapture against the device node.

    /camera/camera_info is not published yet; add it once the lens/sensor
    has been calibrated (see inchiscope_ros2_architecture.md section 5).
    """

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device', '/dev/video0')
        # Without an explicit FOURCC, V4L2 falls back to YUYV, which on this
        # capture card (confirmed via `v4l2-ctl -d /dev/video0
        # --list-formats-ext`) both defaults to a 4:3 800x600 mode and caps
        # well below 30fps at any 16:9 size. MJPG on this same device
        # supports both 1280x720 and 1920x1080 at up to 60fps, so there's no
        # framerate cost to asking for the bigger one.
        self.declare_parameter('pixel_format', 'MJPG')
        # 0/0 means "don't override width/height -- use the capture card's
        # default for whatever pixel_format is set". Only relevant if you
        # change pixel_format to something this device handles differently;
        # left here as an escape hatch rather than the norm.
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'naneye_camera')

        device = self.get_parameter('device').value
        pixel_format = self.get_parameter('pixel_format').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self._frame_id = self.get_parameter('frame_id').value

        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self._capture = cv2.VideoCapture(device)
        if pixel_format:
            self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        if width > 0 and height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._capture.isOpened():
            self.get_logger().error(f'Failed to open capture device {device}')
        else:
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f'Capture resolution: {actual_width}x{actual_height}'
            )

        self._timer = self.create_timer(1.0 / fps, self._grab_and_publish)

    def _grab_and_publish(self):
        if not self._capture.isOpened():
            return
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn('Frame grab failed', throttle_duration_sec=5.0)
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)

    def destroy_node(self):
        if self._capture.isOpened():
            self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
