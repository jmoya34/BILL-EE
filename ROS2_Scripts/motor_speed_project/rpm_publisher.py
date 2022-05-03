#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32

RPM = 5

class rpm(Node):
    def __init__(self):
        super().__init__("rotations_per_min_node")
        self.pub = self.create_publisher(Float32, 'rpm', 10)

        self.timer = self.create_timer(2, self.publish_rpm)
        self.rotations = 5

    def publish_rpm(self):
        num_rotate = Float32()
        num_rotate.data = float(RPM)
        self.pub.publish(num_rotate)


def main():
    rclpy.init()

    my_pub = rpm()

    print("publisher Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown




if __name__ == '__main__':
    main()