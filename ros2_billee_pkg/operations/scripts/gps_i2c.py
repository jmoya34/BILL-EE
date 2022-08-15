#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import smbus

class GPS_Reciver(Node):
    def __init__(self):
        super().__init__("Hello_world_pub_node") #this only takes one argument and it is the name of the node
        self.pub = self.create_publisher(String, 'gps_info', 10)# QOS stand for quality of service
        self.timer = self.create_timer(0.05, self.get_info)

    def get_info(self):
        msg = String()

        i2c_ch = 1
        i2c_address = 0x42
        reg_config = 0xFF

        # Initialize I2C (SMBus)
        bus = smbus.SMBus(i2c_ch)

        # Only here if you want to create a file.
        empty_string = ''
        try: 
            while True:
                val = bus.read_byte_data(i2c_address, reg_config)
                if val > 32 and val == 36:
                    empty_string += "\n" + chr(val)
                elif val == 127:
                    empty_string += " "
                elif val == 255:
                    empty_string = ""    
                elif val > 32:
                    empty_string += chr(val)
                    
                # note the data starts from $GNRMC TO $GNGLL
                if '$GNGSA' in empty_string and '$GNRMC' in empty_string:
                    # print(empty_string[empty_string.find("$GNRMC"):empty_string.find("$GNGSA")])

                    index = empty_string.find("$GNGGA")
                    check1 = empty_string[13:39]
                    check2 = empty_string[index+17:index+43]
                    # check3 = empty_string[20:47]
                            
                    if check1.find(" ") == -1 and check1.find("W") != 1:
                        msg.data = check1
                        print("Cords1:", check1)
                        break
                    elif check2.find(" ") == -1 and index != -1 and check2.find("W") != 1:
                        msg.data = check2
                        print("Cords2:", check2)
                        break
                    # elif check3.find(" ") == -1 and check3.find("W") != 1:
                    #     msg.data = check2
                    #     print("Cords3:", check2)
                    #     break
                    # msg.data = "\n" + empty_string[empty_string.find("$GNRMC"):empty_string.find("$GNGSA")]
                    empty_string = ""
                                
            self.pub.publish(msg)
        except:
            print("[Starting up]")


def main():
    rclpy.init()

    my_pub = GPS_Reciver()

    print("publisher Node Running...")

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()