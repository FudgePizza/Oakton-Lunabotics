import cv2
import numpy as np
import socket
import time


def capture_frame_from_webcam(device_index=0):
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open webcam (index={device_index})")

    ret, frame = cap.read()
    cap.release()
    return frame


def encode_image_to_bytes(image, fmt='.jpg'):
    success, buf = cv2.imencode(fmt, image)
    if not success:
        raise RuntimeError("Failed to encode image")
    return buf.tobytes()


def decode_bytes_to_image(data_bytes):
    arr = np.frombuffer(data_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img

def compress_image(img, quality):
    ret, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ret:
        raise RuntimeError("Failed to encode image")
    decoded_img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return decoded_img


def main():
    total_size = 0.0
    i=0

    # set up webcam
    cap = cv2.VideoCapture(0)

    # set up socket server
    HOST = "0.0.0.0" # Listen on all LAN interfaces
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
    MARKER = b'IMG_END'  # Marker to indicate end of image data

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        conn, addr = s.accept()

        with conn:
            print(f"Connected by {addr}")
            start = time.time()
            while True:
                ret, frame = cap.read()
                if not ret:
                    raise RuntimeError("Failed to read frame from webcam")

                # Compresss frame
                frame = compress_image(frame, quality=5)

                # Encode to bytes
                data_bytes = encode_image_to_bytes(frame, fmt='.jpg')
                imgsize = len(data_bytes) / (1024 * 1024)
                print(f'Encoded image size: {imgsize:.4f} mbytes')
                total_size += imgsize
                i += 1

                # Send image over socket
                conn.sendall(data_bytes + MARKER)  # Append marker to indicate end of image data
                
                if cv2.waitKey(1) == ord('q'):
                    break
        
    end = time.time()

    cap.release()
    cv2.destroyAllWindows()
    print(f'Average: {total_size * 8 / i:.4f} mbytes')
    print(f'Total: {total_size * 8:.4f} mbytes')
    print(f'Time taken: {end - start:.4f} seconds')
    print(f'Mbps: {(total_size * 8) / (end - start):.4f} Mbps')
    print(f'FPS: {i / (end - start):.4f} FPS')


if __name__ == '__main__':
    main()
