from inputs import devices
import time
from control_mapping import XboxController

def findController() -> bool:
    try:
        devices.gamepad
        return True
    except:
        print('No controllers are connected')



def main():
    joy = XboxController()
                           
    while True:
        print(joy.read())
        time.sleep(.01)

main()
print("task ended")