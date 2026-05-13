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
    frame_width = 1280
    frame_height = 960
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width // 2, frame_height // 2))

    HOST = "192.168.0.15"  # IPv4 Address of server computer
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
    # Markers to indicate the end of each image data
    MARKER = b'IMG_END'

    print("Waiting for connection...")

    buffer = b""

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as c:
        c.connect((HOST, PORT))

        print(f"Connected to server at {HOST}:{PORT}")

        start_time = 0
        frame_count = 0
        total_size = 0
        total_time = 0
        maxbandwidth = 0
        try:
            while True:
                #  get data from socket in chunks and append to buffer until we have a complete image
                #c.settimeout(0.5)  # set a timeout to prevent hanging if the server stops sending data
                try:
                    data = c.recv(262144)
                except socket.timeout:
                    data = b"" # no data yet, keep waiting


                if data and len(data) == 0:
                    print("Connection closed by server")
                    break
                buffer += data

                # only proceed if there's at least one complete frame
                if MARKER not in buffer:
                    continue
                
                # Split off all incomplete frames, leave the incomplete tail in the buffer
                parts = buffer.split(MARKER)
                buffer = parts[-1] # parts[-1] is always the incomplete remainder (could be empy b"")

                # if no complete frames, end loop iteration and go again
                complete_frames = parts[:-1] # All other parts are complete image chunks
                if not complete_frames:
                    continue

                # Try to decode from newest to oldest, use first one that succeeds, ignore the rest
                latest_img = None
                latest_img_data = None
                for img_data in reversed(complete_frames):
                    if len(img_data) == 0:
                        continue
                    img = decode_bytes_to_image(img_data)
                    if img is not None:
                        latest_img = img
                        latest_img_data = img_data
                        break
                
                # ensure proper video frame sizing
                target_size = (frame_width, frame_height)
                if (latest_img.shape[1], latest_img.shape[0]) != target_size:
                    latest_img = cv2.resize(latest_img, target_size)
                
                # display frame and write to video
                out.write(latest_img)
                cv2.imshow('Received', latest_img)

                frame_count += 1
                if start_time == 0:
                    start_time = time.time()
                imgsize = len(latest_img_data) * 8/ (1024 * 1024) # in Mbits
                total_size += imgsize
                total_time = time.time() - start_time
                bandwidth = (total_size) / total_time  # in Mbps
                maxbandwidth = max(maxbandwidth, bandwidth)
                fps = frame_count / total_time
                print(f"img size: {imgsize:.4f} Mbits    fps (total): {fps:.4f} fps    bandwidth (total): {bandwidth:.4f} Mbps    max bandwidth: {maxbandwidth:.4f} Mbps")
                
                if cv2.waitKey(1) == ord('q'):
                    break
        finally:
            # Clean up cv2 resources
            out.release()
            cv2.destroyAllWindows()

            print("\nConnection closed.")


if __name__ == "__main__":
    main()
