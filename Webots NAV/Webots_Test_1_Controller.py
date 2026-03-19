# Webots_Test_1_Controller.py
# Main controller for autonomous navigation.
# Coordinates odometry, LiDAR processing, pathfinding, and visualization.
# Controls: SPACE=Start | G=Screenshot | W/A/S/D=Manual

from controller import Robot
import math
import os

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

import odometry
import mapping
import lidar_processing
import pathfinding

# Device names 
LIDAR_NAME       = "lidar"
IMU_NAME         = "inertial unit"
RANGEFINDER_NAME = "ZED_Cam"

MOTOR_NAMES = {
    'FL': "Front_Left_Motor",
    'FR': "Front_Right_Motor",
    'RL': "Rear_Left_Motor",
    'RR': "Rear_Right_Motor",
}

DRIVE_SPEED = 2.0
TURN_SPEED  = 1.5

ROBOT_START_X = 0.6
ROBOT_START_Y = 0.8

GOAL_X = 6.0
GOAL_Y = 0.5

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# Device setup

robot    = Robot()
timestep = int(robot.getBasicTimeStep())
dt       = timestep / 1000.0

print("=" * 60)
print(f"  Lunabotics VFH Controller | dt: {dt}s")
print("=" * 60)

motors = {}
for key, name in MOTOR_NAMES.items():
    m = robot.getDevice(name)
    m.setPosition(float("inf"))
    m.setVelocity(0)
    motors[key] = m

encoders = {}
encoders_ok = True
try:
    for key in MOTOR_NAMES:
        ps = motors[key].getPositionSensor()
        ps.enable(timestep)
        encoders[key] = ps
    print("[OK] Encoders")
except Exception as e:
    encoders_ok = False
    print(f"[ERROR] Encoders: {e}")

imu = None
imu_ok = False
try:
    imu = robot.getDevice(IMU_NAME)
    imu.enable(timestep)
    imu_ok = True
    print("[OK] IMU")
except Exception as e:
    print(f"[ERROR] IMU: {e}")

lidar = None
lidar_ok = False
try:
    lidar = robot.getDevice(LIDAR_NAME)
    lidar.enable(timestep)
    lidar.enablePointCloud()
    lidar_ok = True
    print(f"[OK] LiDAR '{LIDAR_NAME}'")
except Exception as e:
    print(f"[ERROR] LiDAR: {e}")

range_finder = None
rf_ok = False
rf_width = rf_height = 0
try:
    range_finder = robot.getDevice(RANGEFINDER_NAME)
    range_finder.enable(timestep)
    rf_width  = range_finder.getWidth()
    rf_height = range_finder.getHeight()
    rf_ok = True
    print(f"[OK] RangeFinder '{RANGEFINDER_NAME}' ({rf_width}x{rf_height})")
except Exception as e:
    print(f"[WARNING] RangeFinder: {e}")

keyboard = robot.getKeyboard()
keyboard.enable(timestep)


# Module init

odometry.setup(encoders_ok=encoders_ok, imu_ok=imu_ok,
               start_x=ROBOT_START_X, start_y=ROBOT_START_Y)
mapping.setup(script_dir=SCRIPT_DIR, goal_x=GOAL_X, goal_y=GOAL_Y)
if lidar_ok:
    lidar_processing.setup(lidar)
pathfinding.setup(goal_x=GOAL_X, goal_y=GOAL_Y)

print(f"[OK] Start: ({ROBOT_START_X}, {ROBOT_START_Y}) | "
      f"Goal: ({GOAL_X}, {GOAL_Y})")
print(f"[OK] LiDAR: {'ACTIVE' if lidar_ok else 'DISABLED'} | "
      f"Depth: {'ACTIVE' if rf_ok else 'DISABLED'}")
print("Controls: SPACE=Start | G=Screenshot | W/A/S/D=Manual")
print("=" * 60)


# Helpers

def get_depth_front_distance():
    """Min valid distance in the center strip of the depth image."""
    if not rf_ok or not NUMPY_AVAILABLE:
        return float('inf')
    depth_data = range_finder.getRangeImage()
    if depth_data is None:
        return float('inf')

    arr = np.array(depth_data, dtype=np.float32).reshape((rf_height, rf_width))
    row_lo = int(rf_height * 0.30)
    row_hi = int(rf_height * 0.80)
    col_lo = int(rf_width * 0.35)
    col_hi = int(rf_width * 0.65)
    center = arr[row_lo:row_hi, col_lo:col_hi]

    valid = center[(center > 0.1) & (center < 8.0) & np.isfinite(center)]
    if len(valid) == 0:
        return float('inf')
    return float(np.min(valid))


def set_motors(left, right):
    motors['FL'].setVelocity(left)
    motors['RL'].setVelocity(left)
    motors['FR'].setVelocity(right)
    motors['RR'].setVelocity(right)


def stop_motors():
    set_motors(0, 0)


# Main loop

step = 0
autonomous_started = False

while robot.step(timestep) != -1:
    step += 1

    if encoders_ok and imu_ok:
        _, _, yaw = imu.getRollPitchYaw()
        odometry.update(
            encoders['FL'].getValue(), encoders['FR'].getValue(),
            encoders['RL'].getValue(), encoders['RR'].getValue(),
            yaw, dt)

    if lidar_ok:
        cloud = lidar.getPointCloud()
        lidar_processing.update(cloud)

    pathfinding.set_depth_front(get_depth_front_distance())

    mapping.update()

    key = keyboard.getKey()
    manual_override = False

    if key == ord(' '):
        if not autonomous_started:
            pathfinding.start()
            autonomous_started = True
    elif key == ord('W'):
        manual_override = True
        set_motors(DRIVE_SPEED, DRIVE_SPEED)
    elif key == ord('S'):
        manual_override = True
        set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
    elif key == ord('A'):
        manual_override = True
        set_motors(-TURN_SPEED, TURN_SPEED)
    elif key == ord('D'):
        manual_override = True
        set_motors(TURN_SPEED, -TURN_SPEED)

    if not manual_override:
        if autonomous_started:
            left, right = pathfinding.update(step)
            set_motors(left, right)
        else:
            stop_motors()

    if step % 25 == 0:
        rx, ry, rh = odometry.get_pose()
        status = pathfinding.get_status() if autonomous_started else "SPACE to start"
        print(f"Step {step:5d} | ({rx:+5.2f}, {ry:+5.2f}) "
              f"{math.degrees(rh):+6.1f} deg | {status}", end='\r')


# Shutdown

print("\n" + "=" * 60)
rx, ry, _ = odometry.get_pose()
print(f"Final pos: ({rx:.3f}, {ry:.3f}) | Goal: ({GOAL_X:.3f}, {GOAL_Y:.3f})")
dist = math.sqrt((GOAL_X - rx)**2 + (GOAL_Y - ry)**2)
print(f"Distance: {dist:.3f}m")
lms = lidar_processing.get_landmarks()
print(f"Rock landmarks: {len(lms)}")
print("=" * 60)

if mapping.viz:
    mapping.viz.close()