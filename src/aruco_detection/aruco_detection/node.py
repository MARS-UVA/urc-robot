"""Detect ArUco tags and publish an annotated camera image."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.declare_parameter('image_topic', '/front_rgbd/color/image_raw')
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        if not config_file:
            config_file = str(
                Path(get_package_share_directory('startup'))
                / 'config' / 'aruco_tags.yaml')
        config_path = Path(config_file).expanduser()
        try:
            with config_path.open() as stream:
                config = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            raise RuntimeError(f'Could not load {config_path}: {error}') from error
        if not isinstance(config, dict) or not isinstance(
                config.get('dictionary'), str):
            raise ValueError(f'{config_path} must contain a dictionary string')
        dictionary_name = config['dictionary']
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV must include the aruco module')
        if not dictionary_name.startswith('DICT_') or not hasattr(
                cv2.aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name))
        
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary)
            if hasattr(cv2.aruco, 'ArucoDetector') else None
        )
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '~/image', 10)
        image_topic = str(self.get_parameter('image_topic').value)
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Loaded {config_path}; detecting {dictionary_name} on {image_topic}; '
            f'annotated images: {self.publisher.topic_name}')

    def image_callback(self, message):
        """Draw tag borders and IDs, preserving the input timestamp and frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.detector is not None:
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated.header = message.header
            self.publisher.publish(annotated)
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().error(f'Could not process image: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
