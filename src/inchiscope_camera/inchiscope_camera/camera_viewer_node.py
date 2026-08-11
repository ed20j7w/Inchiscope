import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraViewerNode(Node):
    """Shows /camera/image_raw in an OpenCV window.

    Deliberately decoupled from camera_node (which just captures and
    publishes, no display) so the viewer can be run standalone against
    whatever is already publishing that topic, skipped entirely on a
    headless machine, or run alongside other nodes (e.g. Aurora) without
    either depending on the other.
    """

    def __init__(self):
        super().__init__('camera_viewer_node')

        self.declare_parameter('window_name', 'Inchiscope NanEye Camera')
        self._window_name = self.get_parameter('window_name').value

        self._bridge = CvBridge()
        self.should_exit = False

        cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)

        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

    def _on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self._window_name, frame)

        # waitKey also drives the GUI event loop (window repaint, close
        # button) -- it has to run every frame, not just on keypresses.
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):  # 'q' or Esc
            self.should_exit = True
        elif cv2.getWindowProperty(self._window_name, cv2.WND_PROP_VISIBLE) < 1:
            # User clicked the window's close button.
            self.should_exit = True

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
