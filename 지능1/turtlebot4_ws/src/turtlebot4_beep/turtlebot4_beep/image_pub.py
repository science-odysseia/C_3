import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')

        self.publisher_ = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(
            0.033,
            self.publish_image
        )

    def publish_image(self):
        ret, frame = self.cap.read()

        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WebcamPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()