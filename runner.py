import pygame
import serial
import time

ser = serial.Serial('COM12', 9600, timeout=1)  # Update with your port

time.sleep(2)  # Wait for connection to establish

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Controller connected:", joystick.get_name())

while True:
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
        ser.write(b'forward\n')
    elif y_axis > 0.5:
        print("Backward")   # Backward
        ser.write(b'backward\n')
    elif x_axis < -0.5:
        print("Left")   # Left
        ser.write(b'left\n')
    elif x_axis > 0.5:
        print("Right")   # Right
        ser.write(b'right\n')
    elif a_button:
        print("A")   # Stop
        ser.write(b'S\n')
    elif b_button:
        print("B")   # Toggle some feature
    elif x_button:
        print("X")   # Another action
        ser.write(b'dig\n')
    elif y_button:
        print("Y")
        break  # Yet another action
    else:
        ser.write(b'stop\n')

    time.sleep(0.1)