import socket
import numpy as np
import cv2
import time

def decode_bytes_to_image(image_bytes):
    arr = np.frombuffer(image_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img

def main():

    # set up video writer
    cap = cv2.VideoCapture(0)  # don't think we need this, but it gives frame sizes
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width // 2, frame_height // 2))

    HOST = "192.168.1.102"  # IPv4 Address of server computer
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
    MARKER = b'IMG_END'  # Marker to indicate end of image data

    print("Waiting for connection...")

    buffer = b""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as c:
        c.connect((HOST, PORT))

        print(f"Connected to server at {HOST}:{PORT}")
        while True:
            #  get data from socket in chunks and append to buffer until we have a complete image
            data = c.recv(262144)
            if not data:
                print("Connection closed by server")
                break
            buffer += data

            while MARKER in buffer:
                img_data, buffer = buffer.split(MARKER, 1)
                img = decode_bytes_to_image(img_data)
                if img is None:
                    print("Failed to decode image")
                    continue

                target_size = (frame_width, frame_height)

                if (img.shape[1], img.shape[0]) != target_size:
                    img = cv2.resize(img, target_size)
                out.write(img)
                cv2.imshow('Received', img)
                if cv2.waitKey(1) == ord('q'):
                    break

    out.release()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()