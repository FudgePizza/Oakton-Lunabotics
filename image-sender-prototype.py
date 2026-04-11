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
    if not cap.isOpened():
        raise RuntimeError("Cannot open webcam (index=0)")

    # read first frame to determine sizes and initialize writer
    ret, frame = cap.read()
    if not ret or frame is None:
        cap.release()
        raise RuntimeError("Failed to read initial frame from webcam")

    frame_height, frame_width = frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    target_size = (frame_width // 2, frame_height // 2)
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, target_size)

    start = time.time()
    try:
        while True:
            # use the already-read frame for first iteration
            if i == 0:
                current_frame = frame
            else:
                ret, current_frame = cap.read()
                if not ret or current_frame is None:
                    break

            # Compress frame
            compressed = compress_image(current_frame, quality=5)

            # Encode to bytes
            data_bytes = encode_image_to_bytes(compressed, fmt='.jpg')
            imgsize = len(data_bytes) / (1024 * 1024)
            print(f'Encoded image size: {imgsize:.4f} mbytes')
            total_size += imgsize

            # Decode back to image
            reconstructed = decode_bytes_to_image(data_bytes)

            # Ensure the frame matches the VideoWriter size then write
            if reconstructed is None:
                raise RuntimeError("Reconstructed frame is None")
            if (reconstructed.shape[1], reconstructed.shape[0]) != target_size:
                reconstructed = cv2.resize(reconstructed, target_size)
            out.write(reconstructed)

            # show the reconstructed frame
            cv2.imshow('Reconstructed', reconstructed)

            i += 1
            if i >= 10:
                break
    except KeyboardInterrupt:
        pass
    end = time.time()

    # release resources
    out.release()
    cap.release()

    if i == 0:
        print('No frames processed')
        return

    # total_size is in megabytes; convert to megabits for network throughput numbers
    total_megabits = total_size * 8
    avg_megabits_per_frame = total_megabits / i
    duration = end - start
    print(f'Average per frame: {avg_megabits_per_frame:.6f} megabits')
    print(f'Total: {total_megabits:.6f} megabits')
    print(f'Time taken: {duration:.4f} seconds')
    print(f'Mbps: {total_megabits / duration:.4f} Mbps')
    print(f'FPS: {i / duration:.4f} FPS')


if __name__ == '__main__':
    main()
