import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

servoDriver = PCA9685(i2c)
servoDriver.frequency = 330

#servos used: Drfeify 70kg, 330Hz, 500-2500Us

servo1 = servo.Servo(servoDriver.channels[1], min_pulse=500, max_pulse=2500)
servo2 = servo.Servo(servoDriver.channels[2], min_pulse=500, max_pulse=2500)
servo3 = servo.Servo(servoDriver.channels[3], min_pulse=500, max_pulse=2500)

servoList = [servo1,servo2,servo3]

for j in servoList:
    for i in range(180):
        j.angle = i
        time.sleep(0.03)
    for i in range(180):
        j.angle = 180 - i
        time.sleep(0.03)
