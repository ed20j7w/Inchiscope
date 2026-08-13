import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


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

    /camera/camera_info is published from a `camera_info_path` YAML file
    (the ROS CameraInfo layout `scripts/calibrate_camera.py` writes) once
    the cropped feed has been calibrated -- see
    src/inchiscope_camera/scripts/README.md. Left unpublished if
    `camera_info_path` is empty or its `image_width`/`image_height` don't
    match what this node is actually publishing (a common way to end up
    with intrinsics silently mismatched to a resolution/crop change).
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
        # resolutions (1920x1080 down to 640x480) regardless of the sensor's
        # actual ~320x320 content -- most of which are 4:3/5:4, not 16:9.
        # Bench-confirmed: requesting a non-16:9 size (640x480 was tried)
        # makes the squash *worse*, not better, presumably because the
        # card's ISP pads/scales the content correctly only for a 16:9
        # target. Of this device's three 16:9(-ish) sizes -- 1920x1080,
        # 1360x768 (only approximately 16:9, 1.7708 vs exact 1.7778), and
        # 1280x720 (exact 16:9) -- 1280x720 is both the smallest and the
        # only exact one, so it's the right default: least upscale waste
        # available without reintroducing the aspect-driven squash. 0/0
        # means "don't override width/height -- use the capture card's
        # default", an escape hatch if you change pixel_format to something
        # this device handles differently.
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
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

        # Path to a ROS CameraInfo-layout YAML file (what
        # scripts/calibrate_camera.py writes) -- empty means don't publish
        # /camera/camera_info at all. See src/inchiscope_bringup/config/
        # camera_info.yaml for the current calibration.
        self.declare_parameter('camera_info_path', '')

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
        camera_info_path = self.get_parameter('camera_info_path').value

        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self._capture = cv2.VideoCapture(device)
        if pixel_format:
            self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        if width > 0 and height > 0:
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self._crop = None
        self._camera_info = None
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

            published_width, published_height = (
                (crop_width, crop_height) if self._crop is not None else (actual_width, actual_height)
            )
            self._camera_info = self._load_camera_info(
                camera_info_path, published_width, published_height
            )

        self._timer = self.create_timer(1.0 / fps, self._grab_and_publish)

    def _load_camera_info(self, path, expected_width, expected_height):
        if not path:
            return None
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except OSError as exc:
            self.get_logger().error(f'Failed to read camera_info_path {path}: {exc}')
            return None

        if data['image_width'] != expected_width or data['image_height'] != expected_height:
            self.get_logger().error(
                f"camera_info_path {path} was calibrated at "
                f"{data['image_width']}x{data['image_height']}, but this node is publishing "
                f"{expected_width}x{expected_height} -- not publishing /camera/camera_info "
                "since the intrinsics wouldn't match. Recalibrate against the current "
                "resolution/crop, or fix params.yaml to match the calibration."
            )
            return None

        info = CameraInfo()
        info.width = data['image_width']
        info.height = data['image_height']
        info.distortion_model = data['distortion_model']
        info.d = [float(v) for v in data['distortion_coefficients']['data']]
        info.k = [float(v) for v in data['camera_matrix']['data']]
        info.r = [float(v) for v in data['rectification_matrix']['data']]
        info.p = [float(v) for v in data['projection_matrix']['data']]
        self.get_logger().info(f'Loaded camera_info from {path}')
        return info

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
        stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._image_pub.publish(msg)
        if self._camera_info is not None:
            self._camera_info.header.stamp = stamp
            self._camera_info.header.frame_id = self._frame_id
            self._camera_info_pub.publish(self._camera_info)

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
