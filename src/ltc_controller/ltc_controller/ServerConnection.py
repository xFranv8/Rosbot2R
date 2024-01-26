import socket
import struct


class ServerConnection():
    def __init__(self, ip: str, port: int):
        self.__client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__client_socket.connect((ip, port))

    def send_image(self, image) -> tuple[float, float]:
        image_array = image.tobytes()

        self.__client_socket.send(image_array)

        data = self.__client_socket.recv(8)
        data = struct.unpack('ff', data)

        return data[0], data[1]
