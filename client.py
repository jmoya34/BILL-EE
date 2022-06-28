import socket
import time
import pickle

def client(IP :str, port_num: int, decode: str) -> None:
    HEADERSIZE = 10

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP, port_num))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    while True:
        full_msg = b''
        new_msg = True
        while True:
            msg = s.recv(16)
            if new_msg:
                print("new msg len:",msg[:HEADERSIZE])
                msglen = int(msg[:HEADERSIZE])
                new_msg = False

            print(f"full message length: {msglen}")

            full_msg += msg

            print(len(full_msg))

            if len(full_msg)-HEADERSIZE == msglen:
                print("full msg recvd")
                print(full_msg[HEADERSIZE:])
                print(pickle.loads(full_msg[HEADERSIZE:]))
                new_msg = True
                full_msg = b""
                return


if __name__ == "__main__":
    # while True:
    #     client(socket.gethostname(),9999,"utf-8")
    #     time.sleep(.15)

    print(type([1.394, 0.1329, 0.1203, 0.5823]))