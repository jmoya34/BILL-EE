import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)

kit.servo[0].angle = 0
kit.servo[1].angle = 0
kit.servo[2].angle = 0
kit.servo[3].angle = 0

def right_turn():
    for i in range(len(kit.servo)):
        if i % 2 == 0:
            kit.servo[i].angle = 45
        if i % 2 == 1:
            kit.servo[i].angle = 0

def left_turn():
    for i in range(len(kit.servo)):
        if i % 2 == 0:
            kit.servo[i].angle = 0
        if i % 2 == 1:
            kit.servo[i].angle = 45

def reset():
    for i in range(len(kit.servo)):
        kit.servo[i].angle = 0

def main():
    right_turn()
    time.sleep(1)
    reset()
    left_turn()
    time.sleep(1)
    reset()

if __name__ == "__main__":
    main()