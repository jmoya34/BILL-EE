#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import struct
import pickle


class CameraImgSubscriber(Node):

    def __init__(self):
        super().__init__("camera_server_publisher")
        self.pub = self.create_publisher(Image, "camera_img_server", 10)
        self.timer = self.create_timer(.01, self.publish_camera_info)


    def publish_camera_info(self):
        server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        host_name  = socket.gethostname()
        host_ip = socket.gethostbyname(host_name)
        print('HOST IP:',host_ip)
        port = 10050
        socket_address = (host_ip,port)
        print('Socket created')
        server_socket.bind(socket_address)
        print('Socket bind complete')
        server_socket.listen(5)
        print('Socket now listening')

        while True:
            client_socket,addr = server_socket.accept()
            print('Connection from:',addr)
            if client_socket:
                vid = cv2.VideoCapture(0)
                while(vid.isOpened()):
                    img,frame = vid.read()
                    a = pickle.dumps(frame)
                    message = struct.pack("Q",len(a))+a
                    client_socket.sendall(message)
                    cv2.imshow('Sending...',frame)
                    key = cv2.waitKey(10) 
                    if key ==13:
                        client_socket.close()




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