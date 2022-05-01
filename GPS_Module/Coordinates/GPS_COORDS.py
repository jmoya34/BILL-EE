import serial
import pynmea2
import time
def get_coors() -> None:
    ser = serial.Serial("/dev/ttyTHS1",9600)

    while True:
        line = ser.readline()
        line = str(line,encoding='utf-8')
        print(line)
        print(end="")
        if line.startswith('$GNRMC'):
            rmc = pynmea2.parse(line)
            print("Latitude:  ", rmc.latitude)
            print("Longitude: ", rmc.longitude)
        time.sleep(1)


if __name__ == "__main__":
    get_coors()