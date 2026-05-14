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

# Code will likely fail if any of the cameras are not working/plugged in
def main():
    total_size = 0.0
    maxbandwidth = 0.0
    i=0

    # set up webcam
    cap1 = cv2.VideoCapture(4)
    cap2 = cv2.VideoCapture(6)
    cap3 = cv2.VideoCapture(8)
    cap4 = cv2.VideoCapture(10)
    cap5 = cv2.VideoCapture(12)
    quality = 10 # video compression quality in %

    # set up socket server
    HOST = "0.0.0.0" # Listen on all LAN interfaces
    PORT = 65433  # Port to listen on (non-privileged ports are > 1023)

    # Markers to indicate end of each image data
    MARKER1 = b'IMG1_END'
    MARKER2 = b'IMG2_END'
    MARKER3 = b'IMG3_END'
    MARKER4 = b'IMG4_END'
    MARKER5 = b'IMG5_END'

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        conn, addr = s.accept()

        

        with conn:
            print(f"Connected by {addr}")
        
        
            oldrqst = b'-1'
            #start = time.time()
            try:
                while True:
                    ret1, frame1 = cap1.read()
                    ret2, frame2 = cap2.read()
                    ret3, frame3 = cap3.read()
                    ret4, frame4 = cap4.read()
                    ret5, frame5 = cap5.read()
                    if not (ret1 and ret2 and ret3 and ret4 and ret5):
                        raise RuntimeError("Failed to read frame from one or more cams")

                    # Compresss frame
                    frame1 = compress_image(frame1, quality=quality)
                    frame2 = compress_image(frame2, quality=quality)
                    frame3 = compress_image(frame3, quality=quality)
                    frame4 = compress_image(frame4, quality=quality)
                    frame5 = compress_image(frame5, quality=quality)

                    # Encode to bytes
                    data_bytes1 = encode_image_to_bytes(frame1, fmt='.jpg')
                    data_bytes2 = encode_image_to_bytes(frame2, fmt='.jpg')
                    data_bytes3 = encode_image_to_bytes(frame3, fmt='.jpg')
                    data_bytes4 = encode_image_to_bytes(frame4, fmt='.jpg')
                    data_bytes5 = encode_image_to_bytes(frame5, fmt='.jpg')

                    # Calculate size and bandwidth
                    imgsize = (len(data_bytes1) + len(data_bytes2) + len(data_bytes3) + len(data_bytes4) + len(data_bytes5)) * 8 / (1024 * 1024) # in Mbits
                    total_size += imgsize
                    #bandwidth = total_size / (time.time() - start)
                    #axbandwidth = max(bandwidth, maxbandwidth)
                    print(f'Encoded image sizes: {imgsize:.4f} Mbits')     #Average bandwidth: {bandwidth:.4f} Mbps     Highest bandwidth: {maxbandwidth:.4f} Mbps     Time: ' + str(time.time() - start))
                    i += 1
                    
                    # Send image over socket
                    conn.sendall(data_bytes1 + MARKER1)  # Append marker to indicate end of image data
                    conn.sendall(data_bytes2 + MARKER2)
                    conn.sendall(data_bytes3 + MARKER3)
                    conn.sendall(data_bytes4 + MARKER4)
                    conn.sendall(data_bytes5 + MARKER5)
                    #time.sleep(0.05) # pause for 0.05 seconds before capturing and sending again
                    
                    if cv2.waitKey(1) == ord('q'):
                        break
            finally:
                # Clean up socket connection
                conn.close()
                s.close()

                # Clean up cv2 resources
                cap1.release()
                cap2.release()
                cap3.release()
                cap4.release()
                cap5.release()
                cv2.destroyAllWindows()

                # Print final statistics
                #end = time.time()
                print(f'Average: {total_size * 8 / i:.4f} mbytes')
                print(f'Total: {total_size * 8:.4f} mbytes')
                #print(f'Time taken: {end - start:.4f} seconds')
                #print(f'Mbps: {(total_size * 8) / (end - start):.4f} Mbps')
                #print(f'FPS: {i / (end - start):.4f} FPS')


if __name__ == '__main__':
    main()
