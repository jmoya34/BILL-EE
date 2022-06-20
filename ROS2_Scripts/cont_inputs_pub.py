#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

inputs = [3.5, 4.986, 23.09, 42.21, 9.54]

class ContInputPublisher(Node):
    def __init__(self):
        super().__init__("controller_inputs_pub_node")
        self.pub = self.create_publisher(Float32MultiArray, "cont_inputs", 10)
        self.timer = self.create_timer(2, self.publish_cont_inputs)
        self.counter = 0

    def publish_cont_inputs(self):
        msg = Float32MultiArray()
        msg.data = inputs
        self.pub.publish(msg)

def main():
    rclpy.init()

    my_pub = ContInputPublisher()

    print("Controler Inputs Publisher Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
