from roboclaw_3 import Roboclaw
from adafruit_servokit import ServoKit
import math
from inputs import devices
import time
from control_mapping import XboxController



# Rover dimensions
D1 = 1 # Horizontal distance between middle of rover and the corner wheels
D2 = 1 # Vertical distance between the middle of rover and back corner wheels
D3 = 1 # Vertical distance between the middle of the rover and front corner wheels
D4 = 1 # Horzontal distance between the middle of the rover and the center wheels

# Motor speeds
MAX_SPEED = 127 # Speed ranges from 0 - 127
MIN_SPEED = 30

baudrate = 38400

address = [0x81, 0x82, 0x83]
roboclaw1 = Roboclaw("/dev/ttyACM0", baudrate) # Front Wheels, M1 = right, M2 = left
roboclaw2 = Roboclaw("/dev/ttyACM1", baudrate) # Middle Wheels, M1 = right, M2 = left
roboclaw3 = Roboclaw("/dev/ttyACM3", baudrate) # Rear Wheels, M1 = right, M2 = left

roboclaw1.Open()
roboclaw2.Open()
roboclaw3.Open()

# servo initialization
kit = ServoKit(channels = 16)
RADIUS = 1
maxRADIUS = 100 # maximum turning radius
minRADIUS = 20 #

def move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction):
    # roboclaw 0x81 controls 1, 4 (front wheels)
    # roboclaw 0x82 controls 2, 5 (middle wheels)
    # roboclaw 0x83 controls 3, 6 (back wheels)
    if(RADIUS == 0):
      motor_velocities = [speed,speed,speed,speed,speed,speed] # Kerchoo THIS IS GOOD PROGRAMMING I SWEAR!!!
    else:
      motor_velocities = calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed,
                                                direction)
    roboclaw1.ForwardM1(address[0], motor_velocities[3]) # V4
    roboclaw1.ForwardM2(address[0], motor_velocities[0]) # V1
    roboclaw2.ForwardM1(address[1], motor_velocities[4]) # V5
    roboclaw2.ForwardM2(address[1], motor_velocities[1]) # V2
    roboclaw3.ForwardM1(address[2], motor_velocities[5]) # V6
    roboclaw3.ForwardM2(address[2], motor_velocities[2]) # V3
    print("Motor velocities: ", motor_velocities)

def control_servo(D1, D2, D3, D4, RADIUS, direction):
    # if controller values (+) True calculate angle to the right, else calculate angles to the left
    # Calculate servo angle returns list in order of front left, front right, back left, back right
  
    angle_input = calculate_servo_angle(D1, D2, D3, D4, RADIUS, direction) # returns a list
    
    kit.servo[0].angle = 68 # The middle point of each servo is 68*
    kit.servo[1].angle = 68 
    kit.servo[2].angle = 68
    kit.servo[3].angle = 68
    
    # roboclaw 0x81 controls 1, 4 (front wheels)
    # roboclaw 0x82 controls 2, 5 (middle wheels)
    # roboclaw 0x83 controls 3, 6 (back wheels)

    if angle_input[0] > 0:
        kit.servo[0].angle = 68 + (angle_input[0]) 
    else:
      kit.servo[0].angle = 68 - (angle_input[0])

    if angle_input[1] > 0:    
        kit.servo[1].angle = 68 + (angle_input[1])
    else:
      kit.servo[1].angle = 68 - (angle_input[1])
    
    if angle_input[2] > 0:
        kit.servo[2].angle = 68 - (angle_input[2])
    else:
      kit.servo[2].angle = 68 + (angle_input[2])

    if angle_input[3] > 0:
        kit.servo[3].angle = 68 - (angle_input[3])  
    else:
      kit.servo[3].angle = 68 + (angle_input[3])



def calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed, direction):
    # if true moving right, else moving left
    # V1 = FL, V2 = ML, V3 = RL, V4 = FR, V5 = MR, V6 = RR
    if direction == "right":
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

def calculate_servo_angle(D1, D2, D3, D4, RADIUS, direction: bool):
   # A1 = wheel1, A2 = wheel3, A3 = wheel4, A4 = wheel6 etc.
  if direction:
    A1 = math.degrees(math.atan(D3/(RADIUS-D1)))
    A2 = math.degrees(math.atan(D2/(RADIUS-D1)))
    A3 = math.degrees(math.atan(D3/(RADIUS+D1)))
    A4 = math.degrees(math.atan(D2/(RADIUS+D1)))
  else:
    A1 = math.degrees(math.atan(D3/(RADIUS+D1)))
    A2 = math.degrees(math.atan(D2/(RADIUS+D1)))
    A3 = math.degrees(math.atan(D3/(RADIUS-D1)))
    A4 = math.degrees(math.atan(D2/(RADIUS-D1)))

  return [int(A1), int(A2), int(A3), int(A4)] 
  

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

# the right stick inputs should be made so the move_turn method is included already and doesn't have to be called to everytime

joy = XboxController()

while True:
    time.sleep(.10)
    print(joy.read())
    left_stick_controller_input = joy.read()[0]  # controls speed
    right_stick_controller_input = joy.read()[1]  # controls turning
    print(left_stick_controller_input)
    print(right_stick_controller_input)

  # Turning left or right
    RADIUS = maxRADIUS - maxRADIUS * right_stick_controller_input
    if RADIUS < minRADIUS:
      RADIUS  = 0
      
    if right_stick_controller_input > 0.15:
      direction = "right"
      control_servo(D1, D2, D3, D4, RADIUS, direction)
    elif right_stick_controller_input < -0.15:
      direction = "left"
      control_servo(D1, D2, D3, D4, RADIUS, direction)
    else:
      RADIUS = 0
      direction = "forward"
      control_servo(D1, D2, D3, D4, RADIUS, direction)

    #moving just forward and backwards
    if left_stick_controller_input > 0.15:
      speed = int(MAX_SPEED * left_stick_controller_input)
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction) 
    elif left_stick_controller_input < -0.15:
      speed = int(MAX_SPEED * -1 * left_stick_controller_input)
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction)
    else:
      speed = 0
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction)  

    


