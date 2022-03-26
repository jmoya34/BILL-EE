from inputs import devices
import time
from control_mapping import XboxController

def findController() -> bool:   # Doesn't work as of now
    try:
        devices.gamepad
        return True
    except:
        print('No controllers are connected')





if __name__ == '__main__':
    joy = XboxController()
    findController()                     
    while True:             
        print(joy.read())
        time.sleep(.01)

