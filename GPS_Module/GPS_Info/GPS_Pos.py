
import serial
import pynmea2
import time
ser = serial.Serial("/dev/ttyTHS1",9600)

def get_cord():
    while True:
        line = ser.readline()
        print(line)
        line = str(line,encoding='utf-8')
        
       
        if line.startswith('$GNRMC'):
            rmc = pynmea2.parse(line)
            print("Latitude:  ", rmc.latitude)
            print("Longitude: ", rmc.longitude)
        time.sleep(1)
     
if __name__ == "__main__":
    get_cord()

