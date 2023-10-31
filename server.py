import cv2
import multiprocessing as mp
import numpy as np
import pygame
import socket
import struct
import time


def receive_data():
    # Set up server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('10.0.81.65', 12345))  # Replace SERVER_IP with the IP address of the server
    server_socket.listen(1)

    # Accept client connection
    client_socket, client_address = server_socket.accept()
    print('Connected to', client_address)

    print("---------------------------------------------")

    buffer = b''

    while True:
        try:
            # Read v
            """message_len = struct.unpack('!I', client_socket.recv(4))[0]
            data = client_socket.recv(message_len)
            cmd = data.decode('utf-8')
            print(f"Command applied received from the robot: {cmd}")"""

            data = client_socket.recv(1024)
            if not data:
                break
            buffer += data

            # Check if the received data completes the image
            if buffer[-2:] == b'\xff\xd9':
                # Convert the received data back to a cv2 image
                image_array = np.frombuffer(buffer, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

                # Process or display the received image
                cv2.imshow('Received Image', image)
                cv2.waitKey(1)

                # Clear the buffer for the next image
                buffer = b''

            # Read image
            """message_len = struct.unpack('!I', client_socket.recv(4))[0]
            data = client_socket.recv(message_len)

            image_array = np.frombuffer(data, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)"""



            # cv2.imshow('Received Image', image)
            # cv2.waitKey(1)

            # Read lidar
            """message_len = struct.unpack('!I', client_socket.recv(4))[0]
            data = client_socket.recv(message_len)
            lidar = data.decode('utf-8')
            print(f"Lidar: {lidar}")

            # Read imu
            message_len = struct.unpack('!I', client_socket.recv(4))[0]
            data = client_socket.recv(message_len)
            imu = data.decode('utf-8')
            print(f"IMU: {imu}")"""

            print("---------------------------------------------")
            
        except Exception as e:
            print(e)
            continue

        time.sleep(0.5)

    cv2.destroyAllWindows()

    # Close the socket connections
    client_socket.close()
    # server_socket.close()


def send_data():
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

    robot_ip: str = "10.0.74.149"  # Replace ROBOT_IP with the IP address of the robot
    robot_port: int = 6278
    robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    while True:
        try:
            robot_socket.connect((robot_ip, robot_port))
            break
        except Exception as e:
            print(e)
            time.sleep(20)
            continue

    current = time.time_ns()

    i: int = 0
    # TODO: Change video path every time you use this script
    with open("video1/commands.txt", "w") as f:
        while i < 9000:
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
                cmd.append(0.9*y_axis_r)
            else:
                cmd.append(0.0)

            if abs(x_axis_l) > 0.1:
                cmd.append(-x_axis_l)
            else:
                cmd.append(0.0)

            data = struct.pack('ff', *cmd)
            robot_socket.sendall(data)

            # f.write(f"{cmd[0]} {cmd[1]}\n")

            time.sleep(0.5)

    robot_socket.close()


def main():
    p1: mp.Process = mp.Process(target=receive_data)
    p2: mp.Process = mp.Process(target=send_data)

    p1.start()
    p2.start()


if __name__ == '__main__':
    main()
