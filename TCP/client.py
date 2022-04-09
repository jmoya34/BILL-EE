import socket

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  #IPv4 and TCP
client.connect(("192.168.1.86", 9999))

msg = client.recv(1024)
print(msg.decode("utf-8"))