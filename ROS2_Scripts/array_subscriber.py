#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import datetime
from std_msgs.msg import Float64MultiArray

class ContInputSubscriber(Node):
    def __init__(self):
        super().__init__("controller_inputs_sub_node")
        self.sub = self.create_subscription(Float64MultiArray, "cont_inputs", self.subscriber_callback, 10)

    def subscriber_callback(self, pub_msg):
        msg = pub_msg.data
        print(msg, "Date time:", datetime.datetime.now())
        print("Testing first index:", msg[0])


def main():
    rclpy.init()

    my_sub = ContInputSubscriber()

    print("Subscriber Node waiting for data...")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()