#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class pubAndSub(Node):
    def __init__(self):
        super().__init__("publisher_subscriber_node")
        self.sub = self.create_subscription(String, "hello_world", self.subscriber_callback, 10)
        self.pub = self.create_publisher(String, "hybrid_node", 10)

    def subscriber_callback(self, msg):
        new_msg = String()
        new_msg.data = msg.data + " is being added in"
        print(msg.data)
        print(new_msg.data)
        self.pub.publish(new_msg)

def main():
    rclpy.init()

    my_sub = pubAndSub()
    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()