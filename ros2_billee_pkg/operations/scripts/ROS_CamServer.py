#! /usr/bin/env python3
# note that i wrote this in ubuntu 20.04.4 where i use colcon to build packages 
from __future__ import division
import cv2
import numpy as np
import socket
import struct
import math
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
        class FrameSegment(object):
            """ 
            Object to break down image frame segment
            if the size of image exceed maximum datagram size 
            """
            MAX_DGRAM = 2**16
            MAX_IMAGE_DGRAM = MAX_DGRAM - 64 # extract 64 bytes in case UDP frame overflown
            def __init__(self, sock, port, addr="0.0.0.0"): #ipv4 goes here
                self.s = sock
                self.port = port
                self.addr = addr

            def udp_frame(self, img):
                """ 
                Compress image and Break down
                into data segments 
                """
                compress_img = cv2.imencode('.jpg', img)[1]
                dat = compress_img.tobytes()
                size = len(dat)
                count = math.ceil(size/(self.MAX_IMAGE_DGRAM))
                array_pos_start = 0
                while count:
                    array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
                    self.s.sendto(struct.pack("B", count) +
                        dat[array_pos_start:array_pos_end], 
                        (self.addr, self.port)
                        )
                    array_pos_start = array_pos_end
                    count -= 1


        def main():
            """ Top level main function """
            # Set up UDP socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            port = 10050

            fs = FrameSegment(s, port)

            cap = cv2.VideoCapture(-1)
            cap.set(cv2.CAP_FFMPEG,1.0)
            cap.set(cv2.CAP_PROP_FPS,30)
            while (cap.isOpened()):
                _, frame = cap.read()
                fs.udp_frame(frame)
            cap.release()
            cv2.destroyAllWindows()
            s.close()

        if __name__ == "__main__":
            main()

def main():
    rclpy.init()

    my_pub = HelloWorldPublisher()

    print("Camera Server Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()
