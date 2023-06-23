import socket
import cv2
import numpy as np

# Set up server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('yourIP', 8080))  # Replace SERVER_IP with the IP address of the server
server_socket.listen(1)

# Accept client connection
client_socket, client_address = server_socket.accept()
print('Connected to', client_address)

# Initialize buffer to store received data
buffer = b''

# Loop to receive data until the entire image is received
while True:
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

cv2.destroyAllWindows()

# Close the socket connections
# client_socket.close()
server_socket.close()
