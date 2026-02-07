import socket
import time
import serial

HOST = "192.168.1.101"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

buffer = b""

ser = serial.Serial('COM5', 9600, timeout=1)  # Update with your port

time.sleep(2)  # Wait for connection to establish

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
                ser.write(line + b"\n")

                if line == b"close":
                    break
            if line == b"close":
                s.close()
                break
