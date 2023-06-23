import time
import socket
import struct

import pygame

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if any joystick/game controller is connected
if pygame.joystick.get_count() == 0:
    print("No joystick/game controller found.")
    exit()

# Initialize the first joystick/game controller
controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Controller: {controller.get_name()}")

robot_ip: str = "robot_ip"  # Replace ROBOT_IP with the IP address of the robot
robot_port: int = 6278
robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_socket.connect((robot_ip, robot_port))

# Main loop
current = time.time_ns()
while True:
    # delta
    actual = time.time_ns()
    delta = current - actual
    current = actual

    # input reading
    pygame.event.pump()
    x_axis_l = controller.get_axis(0)
    y_axis_r = -controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT)

    cmd = []
    if abs(y_axis_r) > 0.1:
        cmd.append(0.85*y_axis_r)
    else:
        cmd.append(0.0)

    if abs(x_axis_l) > 0.1:
        cmd.append(-0.85*x_axis_l)
    else:
        cmd.append(0.0)

    print(f"V: {cmd[0]}")
    print(f"W: {cmd[1]}")

    data = struct.pack('ff', *cmd)
    robot_socket.sendall(data)

    time.sleep(0.05)