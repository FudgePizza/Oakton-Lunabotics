import socket
import pygame
import serial
import time

HOST = "192.168.1.101"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

time.sleep(2)  # Wait for connection to establish

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller connected:", joystick.get_name())

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()

    

    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break

            pygame.event.pump()  # Update controller state
            x_axis = joystick.get_axis(0)  # Left stick horizontal
            y_axis = joystick.get_axis(1)  # Left stick vertical
            a_button = joystick.get_button(0)
            b_button = joystick.get_button(1)
            x_button = joystick.get_button(2)
            y_button = joystick.get_button(3)

            # Convert analog range (-1.0 to 1.0) into simple movement commands
            if y_axis < -0.5:
                print("Forward")  # Forward
                conn.sendall(b'forward\n')
            elif y_axis > 0.5:
                print("Backward")   # Backward
                conn.sendall(b'backward\n')
            elif x_axis < -0.5:
                print("Left")   # Left
                conn.sendall(b'left\n')
            elif x_axis > 0.5:
                print("Right")   # Right
                conn.sendall(b'right\n')
            elif a_button:
                print("A")   # Stop
                conn.sendall(b'stop\n')
            elif b_button:
                print("B")   # Toggle some feature
            elif x_button:
                print("X")   # Another action
                conn.sendall(b'dig\n')
            elif y_button:
                print("Y")
                break  # Yet another action
            else:
                print("None")
                conn.sendall(b'stop\n')

            time.sleep(0.015)
            
            #conn.sendall(b"Hello")
