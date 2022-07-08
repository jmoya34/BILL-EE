#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import socket
import time 
import pickle


class ContInputPublisher(Node):
    
    # inputs = [float(0), float(0), float(0), float(0)]

    def __init__(self):
        super().__init__("controller_inputs_pub_node")
        self.pub = self.create_publisher(Float64MultiArray, "cont_inputs", 10)
        self.timer = self.create_timer(.15, self.publish_cont_inputs)
        self.counter = 0

    def publish_cont_inputs(self):
        msg = Float64MultiArray()
        try:
            temp_data = self.client(port_num=9999, decode="utf-8") # call to the client code here
            msg.data = [float(temp_data[0]), float(temp_data[1]), float(temp_data[2]), float(temp_data[3])]

            print("Value inside msg is: ", msg.data)
        except:
            msg.data = [float(0),float(0),float(0),float(0)]
            print("Value inside msg is:", msg.data)

        self.pub.publish(msg)

    def client(IP :str, port_num: int, decode: str) -> None:
        HEADERSIZE = 10
        IP = socket.gethostname()

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP, port_num))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

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

                print(len(full_msg))

                if len(full_msg)-HEADERSIZE == msglen:
                    print("full msg recvd")
                    print(full_msg[HEADERSIZE:])
                    print(pickle.loads(full_msg[HEADERSIZE:]))
                    return pickle.loads(full_msg[HEADERSIZE:])

def main():
    rclpy.init()

    my_pub = ContInputPublisher()

    print("Recieving inputs node running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()