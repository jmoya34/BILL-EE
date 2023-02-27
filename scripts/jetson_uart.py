#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import serial
import time


class JetsonUart(Node):
    def __init__(self):
        super().__init__("hello_world_sub_node")
        self.sub = self.create_subscription(Float64MultiArray, "cont_inputs",
                                            self.uart_coms, 10)

        # This piece of code is just to start the actual code worth testing
        # self.sub = self.create_subscription(String, "gps_info",
        #                                     self.uart_coms, 10)

    def uart_coms(self, msg):
        ser = serial.Serial(
            port = '/dev/ttyTHS1',
            baudrate = 9600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout=0.5
            )

        # "\n" Is crucial to the data sending. Without it then it won't work.
        data = b"\n" + str(msg.data).encode()
        # data = b"\n" + str([1.23,1.24,1.25]).encode() # This is to test it out without the need of another node 

        for _ in range(1000):
            print("Sending Data\n")
            print(str(data))
            ser.write(data)

def main():
    rclpy.init()

    my_sub = JetsonUart()

    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown




if __name__ == '__main__':
    main()