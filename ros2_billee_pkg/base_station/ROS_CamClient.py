#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages 
from __future__ import division
import cv2
import numpy as np
import socket
import struct
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
        MAX_DGRAM = 2**16

        def dump_buffer(s):
            """ Emptying buffer frame """
            while True:
                seg, addr = s.recvfrom(MAX_DGRAM)
                print(seg[0])
                if struct.unpack("B", seg[0:1])[0] == 1:
                    print("finish emptying buffer")
                    break

        def main():
            """ Getting image udp frame &
            concate before decode and output image """
            
            # Set up socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(('0.0.0.0', 10050)) #put ipv4 here 
            dat = b''
            dump_buffer(s)

            while True:
                seg, addr = s.recvfrom(MAX_DGRAM)
                if struct.unpack("B", seg[0:1])[0] > 1:
                    dat += seg[1:]
                else:
                    dat += seg[1:]
                    img = cv2.imdecode(np.frombuffer(dat, dtype=np.uint8), 1)
                    if img is not None:
                        cv2.imshow('frame', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    dat = b''

            # cap.release()
            cv2.destroyAllWindows()
            s.close()

        if __name__ == "__main__":
            main()

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
