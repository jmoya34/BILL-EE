#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")
        self.sub = self.create_subscription(String, "hello_world",
                                            self.subscriber_callback, 10)

    
    def subscriber_callback(self, msg):
        print("received: " + msg.data)


def main():
    rclpy.init()

    my_sub = HelloWorldSubscriber()

    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown




if __name__ == '__main__':
    main()