import socket
import struct

import cv2
import numpy as np
import torch
from net import CNNModule


dev: str = 'cpu'


def process_image(image: np.ndarray, model: CNNModule, seq: torch.tensor) -> tuple[float, float]:
    image = np.resize(image, (144, 256, 3))
    numpy_image = np.float32(image) / 255
    tensor = torch.tensor(numpy_image).permute(2, 0, 1)

    seq[0, 0:-1, ...] = seq[0, 1:, ...].clone()
    seq[0, -1, ...] = tensor

    y_hat = model(seq)
    y_hat = y_hat.detach().cpu().numpy()

    return y_hat[0][0][0], y_hat[0][0][1]


def start_socket(ip: str, port: int) -> socket.socket:
    # Set up server socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.listen(1)

    # Accept client connection
    client_socket, client_address = server_socket.accept()
    print('Connected to', client_address)

    return client_socket


def receive_image(server_socket: socket.socket, out_file: int, model: CNNModule, seq: torch.tensor) -> None:
    # Initialize buffer to store received data
    buffer = b''

    # Loop to receive data until the entire image is received
    while True:
        data = server_socket.recv(1024)
        if not data:
            break
        buffer += data

        # Check if the received data completes the image
        if buffer[-2:] == b'\xff\xd9':
            # Convert the received data back to a cv2 image
            image_array = np.frombuffer(buffer, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

            cv2.imshow("", image)
            cv2.waitKey(1)

            print(image.shape)

            vector = np.array(process_image(image, model, seq))

            data = struct.pack('ff', *vector)
            server_socket.sendall(data)

            # Clear the buffer for the next image
            buffer = b''

            return


def main():
    server_socket: socket.socket = start_socket("10.10.1.90", 5009)
    i: int = 0

    hparams = {
        "learning_rate": 5e-3,
        "batch_size": 1,
        "num_workers": 8,
        "num_classes": 2,
        'max_epochs': 300,
        'dropout': 0.2,
        'latent_features': 128,
        'da': True,
    }
    model: CNNModule(hparams) = CNNModule(hparams)
    # model_st_ = torch.load("../models/epoch=12-step=13.ckpt", map_location=torch.device("cpu"))
    # model.load_state_dict(model_st_["state_dict"])
    model = model.to(dev)

    model.eval()

    seq = torch.zeros(1, 1, 3, 144, 256)
    seq = seq.to(dev)

    try:
        while True:
            receive_image(server_socket, i, model, seq)
            i += 1

    except KeyboardInterrupt:
        print("Closing socket...")

        cv2.destroyAllWindows()
        server_socket.close()


if __name__ == '__main__':
    main()
