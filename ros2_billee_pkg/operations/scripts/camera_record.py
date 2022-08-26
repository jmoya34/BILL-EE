#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ContCameraPublisher(Node):

    def __init__(self):
        super().__init__("camera_publisher_node")
        self.pub = self.create_publisher(Image, "camera_img", 10)
        self.timer = self.create_timer(.05, self.publish_camera_info)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def publish_camera_info(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.pub.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')


def main():
    rclpy.init()
    my_pub = ContCameraPublisher()
    print("Camera node running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()