from inputs import devices
from inputs import get_gamepad


def findController():
    try:
        return devices.gamepad
    except:
        print('No controllers are connected')

def totalNumDevices():
     deviceNum = 0
     for device in devices:
        deviceNum += 1
     return deviceNum

def main():
    while 1:
        # events = get_gamepad()
        # for event in events:
        #     print(event.ev_type, event.code, event.state)
        print("test")


findController()

print("task ended")
main()