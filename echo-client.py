import socket
import time

HOST = "192.168.1.101"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

buffer = b""

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"Hello, world")

    while True:
        data = s.recv(1024)
        #if not data:
        #    break
        if data:
            buffer += data

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                print("Received:", line)
        if line == b'close':
            break