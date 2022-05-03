#! /usr/bin/env python3

'''
Lines that involve using a parameter are 22, 33, and 44
'''


import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32

class motor_speed(Node):

    wheel_radius_default = 10 / 100 #in cm

    def __init__(self):
        super().__init__("motor_speed_sub")

        # TODO: THIS IS HOW YOU ADD A PARAMETER
        self.declare_parameter("wheel_radius", self.wheel_radius_default)
        

        self.sub = self.create_subscription(Float32, "rpm", self.calculate_speed, 10)

        self.pub = self.create_publisher(Float32, 'speed', 10)


    def calculate_speed(self, rpm_msg):

        # This gets the wheel data for you to use in your math. the .double_value is import because it turns it from data to a double
        wheel_radius_param = self.get_parameter('wheel_radius').get_parameter_value().double_value

        rotational_speed = rpm_msg.data * 2 * math.pi / 60
        linear_velocity = wheel_radius_param* rotational_speed # we replaced the default wheel radius with the wheel radius parameter in this line
        print("received: " + str(linear_velocity))

        speed_msg = Float32()
        speed_msg.data = float(linear_velocity)
        self.pub.publish(speed_msg)

        # if you want to print out the wheel radius you can do it like this
        # print( self.get_parameter('wheel_radius').get_parameter_value())




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