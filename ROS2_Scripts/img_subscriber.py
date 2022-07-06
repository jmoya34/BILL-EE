#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraImgSubscriber(Node):

    def __init__(self):
        super().__init__("camera_img_subscriber")
        self.sub = self.create_subscription(Image, 'camera_img', self.subscriber_camera_info, 10)
        self.br = CvBridge()

    def subscriber_camera_info(self, msg):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(msg)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)


def main():
    rclpy.init()

    my_pub = CameraImgSubscriber()
    print("Camera node running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()