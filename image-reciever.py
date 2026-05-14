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
    frame_width = 960
    frame_height = 720
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out1 = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    out2 = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    out3 = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    out4 = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))
    out5 = cv2.VideoWriter('output.mp4', fourcc, 20.0, (frame_width, frame_height))

    HOST = "192.168.0.15"  # IPv4 Address of server computer
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
    # Markers to indicate the end of each image data
    MARKER1 = b'IMG1_END'
    MARKER2 = b'IMG2_END'
    MARKER3 = b'IMG3_END'
    MARKER4 = b'IMG4_END'
    MARKER5 = b'IMG5_END'

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
                if MARKER1 not in buffer or MARKER2 not in buffer:
                    continue
                
                # Find indexes of markers
                m1_idx = buffer.find(MARKER1)
                m2_idx = buffer.find(MARKER2, m1_idx + len(MARKER1))
                m3_idx = buffer.find(MARKER3, m2_idx + len(MARKER2))
                m4_idx = buffer.find(MARKER4, m3_idx + len(MARKER3))
                m5_idx = buffer.find(MARKER5, m4_idx + len(MARKER4))


                # Use indexes to update buffers
                img1_data = buffer[:m1_idx]
                img2_data = buffer[m1_idx + len(MARKER1):m2_idx]
                img3_data = buffer[m2_idx + len(MARKER2):m3_idx]
                img4_data = buffer[m3_idx + len(MARKER3):m4_idx]
                img5_data = buffer[m4_idx + len(MARKER4):m5_idx]

                buffer = buffer[m5_idx + len(MARKER5):]

                img1 = decode_bytes_to_image(img1_data) if len(img1_data) > 0 else None
                img2 = decode_bytes_to_image(img2_data) if len(img2_data) > 0 else None
                img3 = decode_bytes_to_image(img3_data) if len(img3_data) > 0 else None
                img4 = decode_bytes_to_image(img4_data) if len(img4_data) > 0 else None
                img5 = decode_bytes_to_image(img5_data) if len(img5_data) > 0 else None
                
                # ensure proper video frame sizing
                target_size = (frame_width, frame_height)
                if (img1.shape[1], img1.shape[0]) != target_size:
                    img1 = cv2.resize(img1, target_size)
                if (img2.shape[1], img2.shape[0]) != target_size:
                    img2 = cv2.resize(img2, target_size)
                if (img3.shape[1], img3.shape[0]) != target_size:
                    img3 = cv2.resize(img3, target_size)
                if (img4.shape[1], img4.shape[0]) != target_size:
                    img4 = cv2.resize(img4, target_size)
                if (img5.shape[1], img5.shape[0]) != target_size:
                    img5 = cv2.resize(img5, target_size)
                                
                # display frame and write to video
                out1.write(img1)
                out2.write(img2)
                out3.write(img3)
                out4.write(img4)
                out5.write(img5)
                cv2.imshow('Camera 1', img1)
                cv2.imshow('Camera 2', img2)
                cv2.imshow('Camera 3', img3)
                cv2.imshow('Camera 4', img4)
                cv2.imshow('Camera 5', img5)


                frame_count += 1
                if start_time == 0:
                    start_time = time.time()
                imgsize = (len(img1_data)+len(img2_data)+len(img3_data)+len(img4_data)+len(img5_data)) * 8/ (1024 * 1024) # in Mbits
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
            out1.release()
            out2.release()
            out3.release()
            out4.release()
            out5.release()
            cv2.destroyAllWindows()

            print("\nConnection closed.")


if __name__ == "__main__":
    main()
