#! /usr/bin/env python3

'''
This can both publish and subscribe the way I wrote it.
This node subscribes to the rpm node and publishes it's own float variable of speed.
I wrote it so it displays the speed and sends information as a publisher.
'''

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32

class motor_speed(Node):

    wheel_radius = 10 / 100 #in cm

    def __init__(self):
        super().__init__("motor_speed_sub")
        self.sub = self.create_subscription(Float32, "rpm", self.calculate_speed, 10)

        self.pub = self.create_publisher(Float32, 'speed', 10)


    def calculate_speed(self, rpm_msg):
        rotational_speed = rpm_msg.data * 2 * math.pi / 60
        linear_velocity = self.wheel_radius * rotational_speed
        print("received: " + str(linear_velocity))

        speed_msg = Float32()
        speed_msg.data = float(linear_velocity)
        self.pub.publish(speed_msg)




def main():
    rclpy.init()

    my_sub = motor_speed()

    print("Waiting for data to be published over topic")

    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown



if __name__ == '__main__':
    main()