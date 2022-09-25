#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__("Hello_world_pub_node") #this only takes one argument and it is the name of the node
        self.pub = self.create_publisher(String, 'hello_world', 10)# QOS stand for quality of service
        self.timer = self.create_timer(2, self.publish_hello_world)
        self.counter = 0

    def publish_hello_world(self):
        msg = String()
        msg.data = 'Hello World ' + str(self.counter)
        self.pub.publish(msg)
        self.counter += 1

def main():
    rclpy.init()

    my_pub = HelloWorldPublisher()

    print("publisher Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()