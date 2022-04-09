import socket

server = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #IPv4 and TCP
server.bind(("192.168.1.86", 9999))

server.listen(5)

while True:
    clientsocket, address = server.accept()
    print(f"Connection from {address} has been made")
    clientsocket.send("You are in my Christian Minecraft Server".encode("utf-8"))
    clientsocket.close()

