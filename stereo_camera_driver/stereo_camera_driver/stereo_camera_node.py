import rclpy
from rclpy.node import Node
import cv2
import yaml

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.bridge = CvBridge()

        # ---------- PARAMETERS ----------
        self.declare_parameter('left_id', 0)
        self.declare_parameter('right_id', 1)
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 200)
        self.declare_parameter('fps', 30)
        self.declare_parameter('left_calib', '')
        self.declare_parameter('right_calib', '')

        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # ---------- GStreamer pipelines ----------
        self.pipeline_left = (
            f"nvarguscamerasrc sensor-id=0 sensor-mode=2 ! "
            f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1, format=NV12 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink"
        )

        self.pipeline_right = (
            f"nvarguscamerasrc sensor-id=1 sensor-mode=2 ! "
            f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1, format=NV12 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink"
        )

        self.capL = cv2.VideoCapture(self.pipeline_left, cv2.CAP_GSTREAMER)
        self.capR = cv2.VideoCapture(self.pipeline_right, cv2.CAP_GSTREAMER)

        if not self.capL.isOpened() or not self.capR.isOpened():
            self.get_logger().error("Could not open stereo cameras")
            rclpy.shutdown()
            return

        # ---------- Publishers ----------
        self.pub_left = self.create_publisher(Image, '/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/right/image_raw', 10)

        self.info_left_pub = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.info_right_pub = self.create_publisher(CameraInfo, '/right/camera_info', 10)

        # ---------- Load calibration ----------
        self.left_info = self.load_camera_info(
            self.get_parameter('left_calib').value
        )
        self.right_info = self.load_camera_info(
            self.get_parameter('right_calib').value
        )

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def load_camera_info(self, path):
        msg = CameraInfo()
        if path == '':
            return msg

        with open(path, 'r') as f:
            calib = yaml.safe_load(f)

        msg.width = calib['image_width']
        msg.height = calib['image_height']
        msg.k = calib['camera_matrix']['data']
        msg.d = calib['distortion_coefficients']['data']
        msg.r = calib['rectification_matrix']['data']
        msg.p = calib['projection_matrix']['data']
        msg.distortion_model = calib['distortion_model']
        msg.header.frame_id = 'camera_link'
        return msg

    def timer_callback(self):
        retL, frameL = self.capL.read()
        retR, frameR = self.capR.read()

        if not retL or not retR:
            return

        # 🔥 SAME AS YOUR SCRIPT
        frameL = cv2.flip(frameL, 0)
        frameR = cv2.flip(frameR, 0)

        msgL = self.bridge.cv2_to_imgmsg(frameL, encoding='bgr8')
        msgR = self.bridge.cv2_to_imgmsg(frameR, encoding='bgr8')

        stamp = self.get_clock().now().to_msg()
        msgL.header.stamp = stamp
        msgR.header.stamp = stamp

        msgL.header.frame_id = 'left_camera'
        msgR.header.frame_id = 'right_camera'

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp

        self.pub_left.publish(msgL)
        self.pub_right.publish(msgR)
        self.info_left_pub.publish(self.left_info)
        self.info_right_pub.publish(self.right_info)


def main():
    rclpy.init()
    node = StereoCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
