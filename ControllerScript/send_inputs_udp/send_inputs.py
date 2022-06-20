import time
import socket
import time
import pickle
from control_mapping import XboxController

def server(IP: str, port_num: int, encode: str, any_obj :any) -> None:
    HEADERSIZE = 10

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((IP, port_num))
    s.listen(5)

    while True:
        # now our endpoint knows about the OTHER endpoint.
        clientsocket, address = s.accept()
        print(f"Connection from {address} has been established.")

        d = any_obj
        msg = pickle.dumps(d)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", encode)+msg
        print(msg)
        clientsocket.send(msg)
        return


if __name__ == '__main__':
    joy = XboxController()                    
    while True:             
        # print(joy.read()) # if server isn't working first debug the controller to ensure the object is correct
        server("192.168.254.73",9999,"utf-8", joy.read()) # The ip address differs between users
        time.sleep(.1)
