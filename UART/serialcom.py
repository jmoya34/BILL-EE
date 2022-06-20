import serial
import time
# Deploy this script on the Raspberry Pi

TIMEOUT = 0.1
OPEN_PORT = None # String input for raspberry pi it is usually
BAUD_RATE = 9600

def read_val(port: str, timeout: int, baudrate: int, enable :bool) -> str: 
    '''
    Read Serial Data In

    This function will read 1 byte of serial data. The output of this function is a string 
    stripped of the new line character so it can be easier to typecast into whatever the 
    user wants
    '''
    if enable:
        ser = serial.Serial(self.port = port, self.timeout = timeout, self.baudrate = baudrate)
        ser.open()

        if ser.is_open: 
            print("Serial port is now open")
        else:
            print("Error: serial port is not detected, does not have permission, or is not open")

        
        data = ser.readline()
        return data.replace('\n','')
    else:
        return


