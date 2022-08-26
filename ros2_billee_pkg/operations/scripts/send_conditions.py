#! /usr/bin/env python3

'''
NOT WORKING AS OF THIS UPLOAD TO REPO
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import pickle


class SendInfoComs(Node):
    def __init__(self):
        super().__init__("send_info")
        self.sub = self.create_subscription(String, "gps_info",
                                            self.subscriber_callback, 10)

    
    def subscriber_callback(self, msg):
        print("received: " + msg.data)

 

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        encode = "utf-8"
        port_num = 9999
        host_name  = socket.gethostname()
        IP = "10.110.177.78"
        print("Host IP:", IP)
        any_obj = msg.data
        HEADERSIZE = 10

        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((IP, port_num))
        s.listen(5)

        while True:
            # now our endpoint knows about the OTHER endpoint.
            clientsocket, address = s.accept()
            print(f"Connection from {address} has been established.")

            d = any_obj
            msg = pickle.dumps(d)
            msg = bytes(f"{len(msg):<{HEADERSIZE}}", encode)+msg
            print(msg)
            clientsocket.send(msg)
            return

    



def main():
    rclpy.init()

    my_sub = SendInfoComs()

    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown




if __name__ == '__main__':
    main()