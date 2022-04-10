import socket
import time
import pickle



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
        


if __name__ == "__main__":
    server("192.168.1.86",9999,"utf-8", {1:"hi", 2: "there"})