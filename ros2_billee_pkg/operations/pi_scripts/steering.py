#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import math
from roboclaw_3 import Roboclaw
import time
from std_msgs.msg import String

class SteeringNode(Node):
    def __init__(self):
        super().__init__("steering_node")
        self.sub = self.create_subscription(Float64MultiArray, "cont_inputs", self.steering, 10)

        # Rover dimensions
        self.D1 = 7.254 # Horizontal distance between middle of rover and the corner wheels
        self.D2 = 10.5 # Vertical distance between the middle of rover and back corner wheels
        self.D3 = 10.5 # Vertical distance between the middle of the rover and front corner wheels
        self.D4 = 10.073 # Horzontal distance between the middle of the rover and the center wheels

        # Motor speeds
        self.MAX_SPEED = 97 # Speed ranges from 0 - 127 (SUBTRACT MAXSPEED BY MIN SPEED AND MAKE THAT YOUR MAX)
        self.MIN_SPEED = 30
        self.address = []

        # Servo Variables
        RADIUS = 1
        self.maxRADIUS = 100 # maximum turning radius
        self.minRADIUS = 18 # CHANGE THIS AFTER DAVID GIVES ROVER DIMENSIONS
        maxAngle = 120

        # Roboclaw initilization
        baudrate = 38400
        self.roboclaw1 = Roboclaw("/dev/ttyACM0", baudrate) # 0x80 Front Wheels, M1 = right, M2 = left
        self.roboclaw2 = Roboclaw("/dev/ttyACM1", baudrate) # 0x82 Middle Wheels, M1 = right, M2 = left
        self.roboclaw3 = Roboclaw("/dev/ttyACM2", baudrate) # 0x81 Rear Wheels, M1 = right, M2 = left
        self.roboclaw1.Open()
        self.roboclaw2.Open()
        self.roboclaw3.Open()

        # Configuring Addresses
        objects = [self.roboclaw1, self.roboclaw2, self.roboclaw3]
        addresses = [0x80, 0x81, 0x82]
        for roboclaw in objects:
            counter = 0
            for address in addresses:
                version = roboclaw.ReadVersion(address)
                if version[0]==True:
                    self.address.append(address)
                    addresses.pop(counter)
                    break
                counter += 1

        #servos used: Drfeify 70kg, 330Hz, 500-2500Us
        i2c = busio.I2C(SCL, SDA)
        servoDriver = PCA9685(i2c)
        servoDriver.frequency = 330
        minPulse = 500
        maxPulse = 2500
        actuationRange = 120
        self.servo1 = servo.Servo(servoDriver.channels[0], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
        self.servo2 = servo.Servo(servoDriver.channels[1], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
        self.servo3 = servo.Servo(servoDriver.channels[2], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
        self.servo4 = servo.Servo(servoDriver.channels[3], min_pulse=minPulse, max_pulse=maxPulse, actuation_range = actuationRange)

    
    def steering(self, msg):
        # print("received:", msg.data)
        right_stick_controller_input = msg.data[0]
        left_stick_controller_input = msg.data[1]
        
        motor_currently_stopped = True
        servo_currently_stopped = True
        
        print("\n\nRight stick input:", right_stick_controller_input)
        print("Right stick input:", right_stick_controller_input)


        # Turning left or right (this only affects the servos)
        RADIUS = self.maxRADIUS - self.maxRADIUS * abs(right_stick_controller_input)
        if RADIUS < self.minRADIUS:
            RADIUS = self.minRADIUS
        # print("radius:", RADIUS)

        # Statements for servos only
        servo_direction = "straight"
        if right_stick_controller_input > 0.15: # 0.15 is the deadzone of the controller to prent unecesarry movements
            direction = "right"
            self.control_servo(self.D1, self.D2, self.D3, self.D4, RADIUS, direction)
            servo_direction = "right"
            servo_currently_stopped = False
        elif right_stick_controller_input < -0.15:
            direction = "left"
            self.control_servo(self.D1, self.D2, self.D3, self.D4, RADIUS, direction)
            servo_direction = "left"
            servo_currently_stopped = False
        else:
            if servo_currently_stopped == True:
                pass
            else:
                RADIUS = self.maxRADIUS
                direction = "forward"
                self.control_servo(self.D1, self.D2, self.D3, self.D4, RADIUS, direction)
                servo_direction = "straight"
                servo_currently_stopped = True

        # Moving just forward and backwards (this only affects the roboclaws)
        if left_stick_controller_input > 0.15 or left_stick_controller_input < -0.15:
            speed = 0.0
            if left_stick_controller_input > 0:
                speed = int(self.MIN_SPEED + left_stick_controller_input * self.MAX_SPEED)
                self.move_turn(self.address, speed, self.D1, self.D2, self.D3, self.D4, RADIUS, direction="forward", servo_direction=servo_direction, maxRADIUS=self.maxRADIUS)
            else:
                speed = int(self.MIN_SPEED + left_stick_controller_input * -1 * self.MAX_SPEED)
                self.move_turn(self.address, speed, self.D1, self.D2, self.D3, self.D4, RADIUS, direction="backward", servo_direction=servo_direction, maxRADIUS=self.maxRADIUS)
            motor_currently_stopped = False
        else:
            if motor_currently_stopped == True:
                pass
            else:
                speed = 0
                self.move_turn(self.address, speed, self.D1, self.D2, self.D3, self.D4, RADIUS, direction="does_not_matter", servo_direction=servo_direction, maxRADIUS=self.maxRADIUS)  
            motor_currently_stopped = True

    def move_turn(self, address, speed, D1, D2, D3, D4, RADIUS, direction, servo_direction, maxRADIUS):
        # roboclaw 0x80 controls 1, 4 (front wheels)
        # roboclaw 0x81 controls 2, 5 (middle wheels)
        # roboclaw 0x82 controls 3, 6 (back wheels)
        reverse = False
        if direction == "backward":
            reverse = True

        if(RADIUS == self.maxRADIUS):
            motor_velocities = [speed,speed,speed,speed,speed,speed] # Kerchoo THIS IS GOOD PROGRAMMING I SWEAR!!!
        else:
            motor_velocities = self.calculate_motor_velocity(self.D1, self.D2, self.D3, self.D4, RADIUS, speed, direction=servo_direction)
        
        if not reverse:
            self.roboclaw1.ForwardM1(self.address[0], motor_velocities[3]) # V4
            self.roboclaw1.ForwardM2(self.address[0], motor_velocities[0]) # V1
            time.sleep(.06)
            self.roboclaw2.ForwardM1(self.address[1], motor_velocities[4]) # V5
            self.roboclaw2.ForwardM2(self.address[1], motor_velocities[1]) # V2
            time.sleep(.06)
            self.roboclaw3.ForwardM1(self.address[2], motor_velocities[5]) # V6
            self.roboclaw3.ForwardM2(self.address[2], motor_velocities[2]) # V3
        else:
            self.roboclaw1.BackwardM1(self.address[0], motor_velocities[3]) # V4
            self.roboclaw1.BackwardM2(self.address[0], motor_velocities[0]) # V1
            time.sleep(.06)
            self.roboclaw2.BackwardM1(self.address[1], motor_velocities[4]) # V5
            self.roboclaw2.BackwardM2(self.address[1], motor_velocities[1]) # V2
            time.sleep(.06)
            self.roboclaw3.BackwardM1(self.address[2], motor_velocities[5]) # V6
            self.roboclaw3.BackwardM2(self.address[2], motor_velocities[2]) # V3
        print("Motor velocities: ", motor_velocities)

    def control_servo(self, D1, D2, D3, D4, RADIUS, direction):
        # if controller values (+) True calculate angle to the right, else calculate angles to the left
        # Calculate servo angle returns list in order of front left, front right, back left, back right
        mid_angle = 60
        if direction == "forward":
            angle_input = [0,0,0,0]
            self.servo1.angle = mid_angle
            self.servo2.angle = mid_angle 
            self.servo3.angle = mid_angle
            self.servo4.angle = mid_angle
            return
        else:
            angle_input = self.calculate_servo_angle(self.D1, self.D2, self.D3, self.D4, RADIUS, direction) # returns a list
            # print("Angle input:", angle_input) # WHEN DEBUGGING THE VALUES SHOULD MAX OUT AT 45

        self.servo1.angle = mid_angle + angle_input[0]
        self.servo2.angle = mid_angle + angle_input[1] 
        self.servo3.angle = mid_angle + angle_input[2]
        self.servo4.angle = mid_angle + angle_input[3]

    def calculate_motor_velocity(self, D1, D2, D3, D4, RADIUS, speed, direction):
        # if true moving right, else moving left
        # V1 = FL, V2 = ML, V3 = RL, V4 = FR, V5 = MR, V6 = RR
        if direction == "right":
            print("calculated moving right")
            V1 = speed * ((math.sqrt((self.D3**2) + (self.D1 + RADIUS)**2)) / (RADIUS + self.D4))
            V2 = speed
            V3 = speed * ((math.sqrt((self.D2**2) + (self.D1 + RADIUS)**2)) / (RADIUS + self.D4))
            V4 = speed * ((math.sqrt((self.D3**2) + (RADIUS - self.D1)**2)) / (RADIUS + self.D4))
            V5 = speed * ((RADIUS - self.D4) / (RADIUS + self.D4))
            V6 = speed * ((math.sqrt((self.D2**2) + (RADIUS - self.D1)**2)) / (RADIUS + self.D4))
        else:
            print("calculated moving left")
            V1 = speed * ((math.sqrt((self.D3**2) + (RADIUS - self.D1)**2)) / (RADIUS + self.D4))
            V2 = speed * ((RADIUS - self.D4) / (RADIUS + self.D4))
            V3 = speed * ((math.sqrt((self.D2**2) + (RADIUS - self.D1)**2)) / (RADIUS + self.D4))
            V4 = speed * ((math.sqrt((self.D3**2) + (self.D1 + RADIUS)**2)) / (RADIUS + self.D4))
            V5 = speed
            V6 = speed * ((math.sqrt((self.D2**2) + (self.D1 + RADIUS)**2)) / (RADIUS + self.D4))

        return [int(V1), int(V2), int(V3), int(V4), int(V5), int(V6)]


    def calculate_servo_angle(self, D1, D2, D3, D4, RADIUS, direction):
        # A1 = wheel1, A2 = wheel3, A3 = wheel4, A4 = wheel6 etc.
        if direction == "right":
            A1 = math.degrees(math.atan(self.D3/(RADIUS-self.D1)))
            A2 = math.degrees(math.atan(self.D2/(RADIUS-self.D1)))
            A3 = math.degrees(math.atan(self.D3/(RADIUS+self.D1)))
            A4 = math.degrees(math.atan(self.D2/(RADIUS+self.D1)))
            A1 = A1*-1
            A2 = A2*-1
            A3=A3*-1
            A4=A4*-1
        elif direction == "left":
            A1 = math.degrees(math.atan(self.D3/(RADIUS+self.D1)))
            A2 = math.degrees(math.atan(self.D2/(RADIUS+self.D1)))
            A3 = math.degrees(math.atan(self.D3/(RADIUS-self.D1)))
            A4 = math.degrees(math.atan(self.D2/(RADIUS-self.D1)))
        
        return [int(A1), int(A2), int(A3), int(A4)] 


def main():
    rclpy.init()
    my_sub = SteeringNode()
    print("Waiting for data to be published over topic")
    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        my_sub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()