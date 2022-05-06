#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
from std_msgs.msg import String, Float32MultiArray

class remote_inputs_node(Node):
    def __init__(self):
        super().__init__("remote_inputs_Node")
        self.pub = self.create_publisher(Float32MultiArray, self.get_input(), 10)
        self.timer = self.create_timer(2, self.time_callback)        


    def time_callback(self) -> None:
        msg = String()
        msg.data = "remote inputs node running"
        self.pub.publish(msg)

    def get_input(self) -> float:
        port_num = 9999
        IP = "192.168.1.86"
        decode = "utf-8"

        HEADERSIZE = 10
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP, port_num))

        while True:
            full_msg = b''
            new_msg = True
            while True:
                print("looking for data")
                msg = s.recv(16)
                if new_msg:
                    print("new msg len:",msg[:HEADERSIZE])
                    msglen = int(msg[:HEADERSIZE])
                    new_msg = False

                print(f"full message length: {msglen}")

                full_msg += msg
                return full_msg
        return full_msg


    def client(IP :str, port_num: int, decode: str) -> float:
        HEADERSIZE = 10

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP, port_num))

        while True:
            full_msg = b''
            new_msg = True
            while True:
                msg = s.recv(16)
                if new_msg:
                    print("new msg len:",msg[:HEADERSIZE])
                    msglen = int(msg[:HEADERSIZE])
                    new_msg = False

                print(f"full message length: {msglen}")

                full_msg += msg
                return full_msg
        return float([0,0,0,0])

def main():
    rclpy.init()

    my_pub = remote_inputs_node()

    print("Inputs Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown



if __name__ == '__main__':
    main()