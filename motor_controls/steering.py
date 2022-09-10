from roboclaw import RoboClaw
from threading import Thread
import math
from inputs import devices
import time
from control_mapping import XboxController

# Rover dimensions
RADIUS = 1
D1 = 1
D2 = 1
D3 = 1
D4 = 1

# Motor speeds
MAX_SPEED = 250
MIN_SPEED = 30

address = [0x81, 0x82, 0x83]
roboclaw = RoboClaw("/dev/ttyS0", 38400)
roboclaw.Open()

roboclaw.ForwardM1(address,
                   int(MAX_SPEED * 0.252))  # 0.252 represents controller input


def move_forward(address, speed):
    for current_speed in range(0, speed + 1):
        roboclaw.ForwardM1(address, current_speed)
        roboclaw.ForwardM2(address, current_speed)


def move_backwards(address, speed):
    for current_speed in range(0, speed + 1):
        roboclaw.BackwardM1(address, current_speed)
        roboclaw.BackwardM2(address, current_speed)


def move_turn(address, speed):
    # Todo: set corners positions later and get direction
    direction = True  # if controller values (+) True, else False

    # roboclaw 0x81 controls 1, 4 (front wheels)
    # roboclaw 0x82 controls 2, 5 (middle wheels)
    # roboclaw 0x83 controls 3, 6 (back wheels)
    motor_velocities = calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed,
                                                direction)

    for single_roboclaw in address:
        Thread(
            target=set_motor_speed(single_roboclaw, motor_velocities)).start()


def set_motor_speed(address, set_speed):
    if address == 0x81:
        roboclaw.ForwardM1(address, set_speed[0])
        roboclaw.ForwardM2(address, set_speed[3])
    elif address == 0x82:
        roboclaw.ForwardM1(address, set_speed[1])
        roboclaw.ForwardM2(address, set_speed[4])
    elif address == 0x83:
        roboclaw.ForwardM1(address, set_speed[2])
        roboclaw.ForwardM2(address, set_speed[5])


def calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed, direction: bool):
    # if true moving right, else moving left

    if direction:
        V1 = speed * ((math.sqrt((D3**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
        V2 = speed
        V3 = speed * ((math.sqrt((D2**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
        V4 = speed * ((math.sqrt((D3**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
        V5 = speed * ((RADIUS - D4) / (RADIUS + D4))
        V6 = speed * ((math.sqrt((D2**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
    else:
        V1 = speed * ((math.sqrt((D3**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
        V2 = speed * ((RADIUS - D4) / (RADIUS + D4))
        V3 = speed * ((math.sqrt((D2**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
        V4 = speed * ((math.sqrt((D3**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
        V5 = speed
        V6 = speed * ((math.sqrt((D2**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))

    return [int(V1), int(V2), int(V3), int(V4), int(V5), int(V6)]


'''
minimun required controller has
left stick, right stick 
the inputs library will output a list in the following format
leftstick's x position, leftstick's y position, rightstick's x position, rightstick's y position

which will be in a list and will range from -1 to 1
example of this is 
[.750000, -0.631212, .500000, 0.000000]
so the variable left_stick_controller_input will be arr[0]
the scheme i believe we are going for is like a traditional rc car
so arr[0] will control speed 
and arr[-1] will control turning
'''

left_stick_controller_input = 1  # controls speed
right_stick_controller_input = 0  # controls turning

# the right stick inputs should be made so the move_turn method is included already and doesn't have to be called to everytime
if left_stick_controller_input > 0:
    move_forward(address, int(MAX_SPEED * left_stick_controller_input))
else:
    move_backwards(address,
                   int(MAX_SPEED * math.abs(left_stick_controller_input)))

if right_stick_controller_input > 0:
    move_turn(address, int(MAX_SPEED * right_stick_controller_input))
else:
    move_turn(address, int(MAX_SPEED * math.abs(right_stick_controller_input)))

joy = XboxController()                 
while True:             
    print(joy.read())
    time.sleep(.01)