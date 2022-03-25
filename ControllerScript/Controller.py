from inputs import devices
from inputs import get_gamepad
import time


def findController():
    try:
        devices.gamepad()
        return True
    except:
        print('No controllers are connected')

def totalNumDevices():
     deviceNum = 0
     for device in devices:
        deviceNum += 1
     return deviceNum

def main():
    while 1:
        events = get_gamepad()
        for event in events:
            print(event.ev_type, event.code, event.state)
            print(devices.xinput, devices.xinput_dll)
            print(devices.yinput, devices.yinput_dll)
    time.sleep(1)


if findController():
    main()

# print(dir(devices.gamepads))
print("task ended")
