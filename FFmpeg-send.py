import subprocess
import sys

CONTROLLER_IP = "192.168.1.50"   # CHANGE THIS
PORT = 5000

# -------- OS DETECTION --------
if sys.platform.startswith("win"):
    CAMERA_INPUT = [
        "-f", "dshow",
        "-i", "video=Integrated Camera"  # CHANGE camera name if needed
    ]
else:
    CAMERA_INPUT = [
        "-f", "v4l2",
        "-i", "/dev/video0"              # CHANGE if camera index differs
    ]

# -------- FFMPEG COMMAND --------
ffmpeg_cmd = [
    "ffmpeg",

    *CAMERA_INPUT,

    # Video settings
    "-vf", "scale=320:240", # Size of video stream output
    "-r", "15",             # Frame rate

    # Encoder
    "-c:v", "libx264",      # H.264 encoding
    "-preset", "ultrafast", # Fast encoding, low latency
    "-tune", "zerolatency", # Optimize for low latency
    "-b:v", "1M",           # Bitrate (adjust as needed)
    "-maxrate", "1M",       # Max bitrate (comp is 4Mb, but we want to keep it low for low latency)
    "-bufsize", "1M",       # Buffer size
    "-g", "15",             # Keyframe interval
    "-bf", "0",             # No B-frames

    # No audio
    "-an",

    # Output over UDP
    "-f", "mpegts",
    f"udp://{CONTROLLER_IP}:{PORT}"
]

print("Starting video stream...")
subprocess.run(ffmpeg_cmd)