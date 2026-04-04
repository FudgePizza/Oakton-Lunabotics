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

    # set up webcam and video writer
    cap = cv2.VideoCapture(0)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width // 2, frame_height // 2))

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

        # Decode back to image
        reconstructed = decode_bytes_to_image(data_bytes)

        # Ensure the frame matches the VideoWriter size then write
        target_size = (frame_width // 2, frame_height // 2)
        if reconstructed is None:
            raise RuntimeError("Reconstructed frame is None")
        if (reconstructed.shape[1], reconstructed.shape[0]) != target_size:
            reconstructed = cv2.resize(reconstructed, target_size)
        out.write(reconstructed)

        # Show reconstructed image (larger display but without increasing quality)
        display_scale = 2
        display_size = (target_size[0] * display_scale, target_size[1] * display_scale)
        display_img = cv2.resize(reconstructed, display_size, interpolation=cv2.INTER_NEAREST)
        cv2.imshow('reconstructed', display_img)

        i+=1
        if cv2.waitKey(1) == ord('q'):
            break
        if i >= 500:
            break
    end = time.time()

    out.release()
    cap.release()
    cv2.destroyAllWindows()
    print(f'Average: {total_size * 8 / i:.4f} mbytes')
    print(f'Total: {total_size * 8:.4f} mbytes')
    print(f'Time taken: {end - start:.4f} seconds')
    print(f'Mbps: {(total_size * 8) / (end - start):.4f} Mbps')
    print(f'FPS: {i / (end - start):.4f} FPS')


if __name__ == '__main__':
    main()
