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

    The capture card's own ISP pads the sensor's true ~320x320 content with
    a black border (plus a logo/info overlay in part of that border) to
    whichever fixed resolution is requested, then scales the whole padded
    frame -- border included -- up to that size. None of that border is
    real image data, so left uncropped it's pure wasted pixels through
    every downstream CV step (feature detection, undistortion, display).
    `crop_*` below cuts it out; see inchiscope_camera/README.md for how to
    find the right values for your capture card and mounting.

    /camera/camera_info is not published yet; add it once the cropped feed
    has been calibrated (see inchiscope_ros2_architecture.md section 5) --
    calibrate against the *cropped* output, not the raw padded frame.
    """

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device', '/dev/video0')
        # Without an explicit FOURCC, V4L2 falls back to YUYV, which on this
        # capture card caps out well below 30fps at anything but the
        # smallest sizes. MJPG hits 60fps at every size this device offers
        # (confirmed via `v4l2-ctl -d /dev/video0 --list-formats-ext`).
        self.declare_parameter('pixel_format', 'MJPG')
        # This capture card only advertises a fixed set of generic "webcam"
        # resolutions (1920x1080 down to 640x480, plus some 4:3/5:4 sizes in
        # between) regardless of the sensor's actual ~320x320 content --
        # 640x480 is the smallest it offers, so it's the least-upscaled
        # option available; anything bigger is pure waste (more pixels for
        # every downstream CV op, zero extra real detail). 0/0 means "don't
        # override width/height -- use the capture card's default", an
        # escape hatch if you change pixel_format to something this device
        # handles differently.
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'naneye_camera')

        # Manual crop to cut the black border (and logo overlay) out of the
        # published image -- disabled (crop_width/crop_height == 0) until
        # set, since part of the border isn't pure black (a logo/info
        # overlay lives in it), which breaks a simple auto-detect-the-black-
        # border approach. Determine these by eye against a saved frame at
        # the resolution above (see README) and set all four here.
        self.declare_parameter('crop_x', 0)
        self.declare_parameter('crop_y', 0)
        self.declare_parameter('crop_width', 0)
        self.declare_parameter('crop_height', 0)

        device = self.get_parameter('device').value
        pixel_format = self.get_parameter('pixel_format').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self._frame_id = self.get_parameter('frame_id').value
        crop_x = int(self.get_parameter('crop_x').value)
        crop_y = int(self.get_parameter('crop_y').value)
        crop_width = int(self.get_parameter('crop_width').value)
        crop_height = int(self.get_parameter('crop_height').value)

        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self._capture = cv2.VideoCapture(device)
        if pixel_format:
            self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        if width > 0 and height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self._crop = None
        if not self._capture.isOpened():
            self.get_logger().error(f'Failed to open capture device {device}')
        else:
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f'Capture resolution: {actual_width}x{actual_height}'
            )
            if crop_width > 0 and crop_height > 0:
                if crop_x + crop_width > actual_width or crop_y + crop_height > actual_height:
                    self.get_logger().error(
                        f'crop region ({crop_x},{crop_y},{crop_width}x{crop_height}) '
                        f'does not fit inside the {actual_width}x{actual_height} capture '
                        '-- publishing uncropped frames instead'
                    )
                else:
                    self._crop = (crop_x, crop_y, crop_width, crop_height)
                    self.get_logger().info(f'Cropping to {crop_width}x{crop_height} at ({crop_x},{crop_y})')

        self._timer = self.create_timer(1.0 / fps, self._grab_and_publish)

    def _grab_and_publish(self):
        if not self._capture.isOpened():
            return
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warn('Frame grab failed', throttle_duration_sec=5.0)
            return
        if self._crop is not None:
            x, y, w, h = self._crop
            frame = frame[y:y + h, x:x + w]
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
