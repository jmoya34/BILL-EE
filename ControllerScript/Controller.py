from inputs import devices
import time
from control_mapping import XboxController

deadzone = 4/100        #4% deadzone 

def findController():
    try:
        devices.gamepad
        return True
    except:
        print('No controllers are connected')


def setDeadZone(deadZone:int):
    deadzone = deadZone / 100


def calibration(info: list):
    tempList = []
    for data in info:
        if data < deadzone:
            tempList.append(0)
        else:
            tempList.append(data)
    return tempList


def main():
    joy = XboxController()
    while True:
        print(calibration(joy.read()) )
        print(joy.read() )
        time.sleep(.01)

main()
print("task ended")