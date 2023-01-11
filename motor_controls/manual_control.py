from roboclaw_3 import Roboclaw
import math
from inputs import devices
import time
from control_mapping import XboxController
from datetime import datetime
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Rover dimensions
D1 = 7.254 # Horizontal distance between middle of rover and the corner wheels
D2 = 10.5 # Vertical distance between the middle of rover and back corner wheels
D3 = 10.5 # Vertical distance between the middle of the rover and front corner wheels
D4 = 10.073 # Horzontal distance between the middle of the rover and the center wheels

# Motor speeds
MAX_SPEED = 97 # Speed ranges from 0 - 127 (SUBTRACT MAXSPEED BY MIN SPEED AND MAKE THAT YOUR MAX)
MIN_SPEED = 30

# Roboclaw initilization
baudrate = 38400
address = [0x80, 0x82, 0x81]
roboclaw1 = Roboclaw("/dev/ttyACM0", baudrate) # 0x80 Front Wheels, M1 = right, M2 = left
roboclaw2 = Roboclaw("/dev/ttyACM1", baudrate) # 0x82 Middle Wheels, M1 = right, M2 = left
roboclaw3 = Roboclaw("/dev/ttyACM2", baudrate) # 0x81 Rear Wheels, M1 = right, M2 = left
roboclaw1.Open()
roboclaw2.Open()
roboclaw3.Open()

#servos used: Drfeify 70kg, 330Hz, 500-2500Us
i2c = busio.I2C(SCL, SDA)
servoDriver = PCA9685(i2c)
servoDriver.frequency = 330
minPulse = 500
maxPulse = 2500
actuationRange = 120
servo1 = servo.Servo(servoDriver.channels[0], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
servo2 = servo.Servo(servoDriver.channels[1], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
servo3 = servo.Servo(servoDriver.channels[2], min_pulse=minPulse, max_pulse=maxPulse ,actuation_range = actuationRange)
servo4 = servo.Servo(servoDriver.channels[3], min_pulse=minPulse, max_pulse=maxPulse, actuation_range = actuationRange)


def move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction, servo_direction):
  # roboclaw 0x80 controls 1, 4 (front wheels)
  # roboclaw 0x81 controls 2, 5 (middle wheels)
  # roboclaw 0x82 controls 3, 6 (back wheels)
  reverse = False
  if direction == "backward":
    reverse = True

  if(RADIUS == maxRADIUS):
    motor_velocities = [speed,speed,speed,speed,speed,speed] # Kerchoo THIS IS GOOD PROGRAMMING I SWEAR!!!
  else:
    motor_velocities = calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed, direction=servo_direction)
  
  if not reverse:
    roboclaw1.ForwardM1(address[0], motor_velocities[3]) # V4
    roboclaw1.ForwardM2(address[0], motor_velocities[0]) # V1
    time.sleep(.06)
    roboclaw2.ForwardM1(address[1], motor_velocities[4]) # V5
    roboclaw2.ForwardM2(address[1], motor_velocities[1]) # V2
    time.sleep(.06)
    roboclaw3.ForwardM1(address[2], motor_velocities[5]) # V6
    roboclaw3.ForwardM2(address[2], motor_velocities[2]) # V3
  else:
    roboclaw1.BackwardM1(address[0], motor_velocities[3]) # V4
    roboclaw1.BackwardM2(address[0], motor_velocities[0]) # V1
    time.sleep(.06)
    roboclaw2.BackwardM1(address[1], motor_velocities[4]) # V5
    roboclaw2.BackwardM2(address[1], motor_velocities[1]) # V2
    time.sleep(.06)
    roboclaw3.BackwardM1(address[2], motor_velocities[5]) # V6
    roboclaw3.BackwardM2(address[2], motor_velocities[2]) # V3
  print("Motor velocities: ", motor_velocities)

def control_servo(D1, D2, D3, D4, RADIUS, direction):
  # if controller values (+) True calculate angle to the right, else calculate angles to the left
  # Calculate servo angle returns list in order of front left, front right, back left, back right
  mid_angle = 60
  if direction == "forward":
    angle_input = [0,0,0,0]
    servo1.angle = mid_angle
    servo2.angle = mid_angle 
    servo3.angle = mid_angle
    servo4.angle = mid_angle
    return
  else:
    angle_input = calculate_servo_angle(D1, D2, D3, D4, RADIUS, direction) # returns a list
    # print("Angle input:", angle_input) # WHEN DEBUGGING THE VALUES SHOULD MAX OUT AT 45

  servo1.angle = mid_angle + angle_input[0]
  servo2.angle = mid_angle + angle_input[1] 
  servo3.angle = mid_angle + angle_input[2]
  servo4.angle = mid_angle + angle_input[3]



def calculate_motor_velocity(D1, D2, D3, D4, RADIUS, speed, direction):
  # if true moving right, else moving left
  # V1 = FL, V2 = ML, V3 = RL, V4 = FR, V5 = MR, V6 = RR
  if direction == "right":
    print("calculated moving right")
    V1 = speed * ((math.sqrt((D3**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
    V2 = speed
    V3 = speed * ((math.sqrt((D2**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
    V4 = speed * ((math.sqrt((D3**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
    V5 = speed * ((RADIUS - D4) / (RADIUS + D4))
    V6 = speed * ((math.sqrt((D2**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
  else:
    print("calculated moving left")
    V1 = speed * ((math.sqrt((D3**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
    V2 = speed * ((RADIUS - D4) / (RADIUS + D4))
    V3 = speed * ((math.sqrt((D2**2) + (RADIUS - D1)**2)) / (RADIUS + D4))
    V4 = speed * ((math.sqrt((D3**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))
    V5 = speed
    V6 = speed * ((math.sqrt((D2**2) + (D1 + RADIUS)**2)) / (RADIUS + D4))

  return [int(V1), int(V2), int(V3), int(V4), int(V5), int(V6)]


def calculate_servo_angle(D1, D2, D3, D4, RADIUS, direction):
  # A1 = wheel1, A2 = wheel3, A3 = wheel4, A4 = wheel6 etc.
  if direction == "right":
    A1 = math.degrees(math.atan(D3/(RADIUS-D1)))
    A2 = math.degrees(math.atan(D2/(RADIUS-D1)))
    A3 = math.degrees(math.atan(D3/(RADIUS+D1)))
    A4 = math.degrees(math.atan(D2/(RADIUS+D1)))
    A1 = A1*-1
    A2 = A2*-1
    A3=A3*-1
    A4=A4*-1
    # print("A1: ", A1) # Comment out for debugging
    # print("A2: ", A2)
    # print("A3: ", A3)
    # print("A4: ", A4)
    # print("")
  elif direction == "left":
    A1 = math.degrees(math.atan(D3/(RADIUS+D1)))
    A2 = math.degrees(math.atan(D2/(RADIUS+D1)))
    A3 = math.degrees(math.atan(D3/(RADIUS-D1)))
    A4 = math.degrees(math.atan(D2/(RADIUS-D1)))
    # print("A1: ", A1) # Comment out for debugging
    # print("A2: ", A2)
    # print("A3: ", A3)
    # print("A4: ", A4)
    # print("")
  
  return [int(A1), int(A2), int(A3), int(A4)] 
  

# the right stick inputs should be made so the move_turn method is included already and doesn't have to be called to everytime
joy = XboxController()
motor_currently_stopped = True
servo_currently_stopped = True
RADIUS = 1
maxRADIUS = 100 # maximum turning radius
minRADIUS = 18 # CHANGE THIS AFTER DAVID GIVES ROVER DIMENSIONS
maxAngle = 120

while True:
  time.sleep(.10)
  left_stick_controller_input = joy.read()[0]  # controls speed
  right_stick_controller_input = joy.read()[1]  # controls turning
  # print(joy.read())
  # print(left_stick_controller_input)
  # print(right_stick_controller_input)

  # Turning left or right (this only affects the servos)
  RADIUS = maxRADIUS - maxRADIUS * abs(right_stick_controller_input)
  if RADIUS < minRADIUS:
    RADIUS = minRADIUS
  # print("radius:", RADIUS)

  # Statements for servos only
  servo_direction = "straight"
  if right_stick_controller_input > 0.15: # 0.15 is the deadzone of the controller to prent unecesarry movements
    direction = "right"
    control_servo(D1, D2, D3, D4, RADIUS, direction)
    servo_direction = "right"
    servo_currently_stopped = False
  elif right_stick_controller_input < -0.15:
    direction = "left"
    control_servo(D1, D2, D3, D4, RADIUS, direction)
    servo_direction = "left"
    servo_currently_stopped = False
  else:
    if servo_currently_stopped == True:
      pass
    else:
      RADIUS = maxRADIUS
      direction = "forward"
      control_servo(D1, D2, D3, D4, RADIUS, direction)
      servo_direction = "straight"
      servo_currently_stopped = True


  # Moving just forward and backwards (this only affects the roboclaws)
  if left_stick_controller_input > 0.15 or left_stick_controller_input < -0.15:
    speed = 0.0
    if left_stick_controller_input > 0:
      speed = int(MIN_SPEED + left_stick_controller_input * MAX_SPEED)
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction="forward", servo_direction=servo_direction)
    else:
      speed = int(MIN_SPEED + left_stick_controller_input * -1 * MAX_SPEED)
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction="backward", servo_direction=servo_direction)
    motor_currently_stopped = False
  else:
    if motor_currently_stopped == True:
      pass
    else:
      speed = 0
      move_turn(address, speed, D1, D2, D3, D4, RADIUS, direction="does_not_matter", servo_direction=servo_direction)  
      motor_currently_stopped = True