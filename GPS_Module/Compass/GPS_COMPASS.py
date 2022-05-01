import time
import board
import adafruit_lis3mdl

i2c = board.I2C()  # uses board.SCL and board.SDA 
sensor = adafruit_lis3mdl.LIS3MDL(i2c) # The default address is 0x1C
#however the gps module has a address of 0x1E if you connect correctly
#if you want to check if its connected correctly run the folllowing command
#sudo i2cdetect -r -y 0

while True:
    mag_x, mag_y, mag_z = sensor.magnetic

    print('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))
    print('')
    time.sleep(1.0)