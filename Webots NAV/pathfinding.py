# todo: fix obstacle avoidance
import math
import odometry
import lidar_processing

_goal_x = 5.0
_goal_y = 3.0
GOAL_TOLERANCE = 0.20

DRIVE_SPEED = 2.0
TURN_SPEED  = 1.5
SLOW_SPEED  = 1.0

DEPTH_STOP_DIST = 0.5
DEPTH_SLOW_DIST = 1.0

BACKUP_STEPS = 40
SCAN_STEPS   = 65
COMMIT_DIST  = 1.0

STALL_CHECK_INTERVAL = 40
STALL_DIST_THRESHOLD = 0.05

MIN_SIDE_CLEARANCE = 1.0

_state = "IDLE"
_steer_angle = 0.0
_depth_front = float('inf')

_backup_timer = 0
_scan_timer = 0
_scan_direction = 1
_backup_heading = 0.0

_commit_start_x = 0.0
_commit_start_y = 0.0

_stall_x = 0.0
_stall_y = 0.0
_stall_counter = 0


def setup(goal_x=5.0, goal_y=3.0):
    global _goal_x, _goal_y
    _goal_x = goal_x
    _goal_y = goal_y


def start():
    global _state
    _state = "NAVIGATING"
    print(f"[NAV] VFH navigation started, goal ({_goal_x:.2f}, {_goal_y:.2f})")


def set_depth_front(dist):
    global _depth_front
    _depth_front = dist


def update(step):
    global _state, _steer_angle, _backup_timer, _scan_timer
    global _scan_direction, _backup_heading
    global _stall_x, _stall_y, _stall_counter
    global _commit_start_x, _commit_start_y

    rx, ry, rh = odometry.get_pose()

    dist_to_goal = math.sqrt((_goal_x - rx)**2 + (_goal_y - ry)**2)
    if dist_to_goal < GOAL_TOLERANCE:
        if _state != "ARRIVED":
            _state = "ARRIVED"
            print(f"\n  Goal reached at ({rx:.3f}, {ry:.3f})\n")
        return (0, 0)

    if _state in ("IDLE", "ARRIVED"):
        return (0, 0)

    # BACKUP - reverse away from obstacle
    if _state == "BACKUP":
        _backup_timer += 1
        if _backup_timer >= BACKUP_STEPS:
            _backup_timer = 0
            _state = "CHOOSING"
        return (-SLOW_SPEED, -SLOW_SPEED)

    # CHOOSING - check left vs right clearance in VFH
    if _state == "CHOOSING":
        direction = _choose_avoidance_direction(rh)
        if direction is None:
            print("[NAV] STUCK, no clearance on either side")
            _state = "STUCK"
            return (0, 0)
        _scan_direction = direction
        _scan_timer = 0
        _state = "SCAN"
        side = "LEFT" if direction > 0 else "RIGHT"
        print(f"[NAV] Turning {side}")
        return (0, 0)

    # SCAN - turn toward chosen direction
    if _state == "SCAN":
        _scan_timer += 1
        if _scan_timer >= SCAN_STEPS:
            _scan_timer = 0
            _commit_start_x, _commit_start_y = rx, ry
            _state = "COMMIT"
            print("[NAV] Driving straight to clear obstacle")
        t = TURN_SPEED * 0.8 * _scan_direction
        return (-t, t)

    # COMMIT - drive straight past the obstacle, depth camera stays active
    if _state == "COMMIT":
        if _depth_front < DEPTH_STOP_DIST:
            _trigger_backup(rh, step)
            print(f"[NAV] Obstacle during commit at {_depth_front:.2f}m, backing up")
            return (0, 0)

        dist_driven = math.sqrt((rx - _commit_start_x)**2 +
                                (ry - _commit_start_y)**2)
        if dist_driven >= COMMIT_DIST:
            _state = "NAVIGATING"
            _stall_x, _stall_y = rx, ry
            _stall_counter = 0
            print(f"[NAV] Cleared obstacle ({dist_driven:.2f}m), resuming")
        return (DRIVE_SPEED, DRIVE_SPEED)

    # STUCK - wait and retry periodically
    if _state == "STUCK":
        _stall_counter += 1
        if _stall_counter >= 100:
            _stall_counter = 0
            _state = "CHOOSING"
        return (0, 0)

    # NAVIGATING - normal VFH steering toward goal
    if _state == "NAVIGATING":
        lidar_front = lidar_processing.get_front_distance()
        front_dist = min(_depth_front, lidar_front)

        if front_dist < DEPTH_STOP_DIST:
            _trigger_backup(rh, step)
            print(f"[NAV] Obstacle at {front_dist:.2f}m "
                  f"(depth={_depth_front:.2f}, lidar={lidar_front:.2f}), backing up")
            return (0, 0)

        goal_angle = math.atan2(_goal_y - ry, _goal_x - rx)
        goal_err = abs(_angle_diff(goal_angle, rh))

        # Far from goal heading: turn in place, reset stall counter
        if goal_err >= math.radians(60):
            _stall_x, _stall_y = rx, ry
            _stall_counter = 0
            d = 1 if _angle_diff(goal_angle, rh) > 0 else -1
            return (-TURN_SPEED * d * 0.6, TURN_SPEED * d * 0.6)

        # Stall detection (only when robot is driving forward)
        _stall_counter += 1
        if _stall_counter >= STALL_CHECK_INTERVAL:
            moved = math.sqrt((rx - _stall_x)**2 + (ry - _stall_y)**2)
            if moved < STALL_DIST_THRESHOLD:
                _trigger_backup(rh, step)
                print(f"[NAV] Stall detected (moved {moved:.3f}m), backing up")
                return (0, 0)
            _stall_x, _stall_y = rx, ry
            _stall_counter = 0

        sector_angles, sector_dists = lidar_processing.get_histogram()
        if not sector_angles:
            return _steer_toward(rh, goal_angle, SLOW_SPEED)

        best_angle = _find_best_gap(sector_angles, sector_dists, goal_angle)

        if best_angle is not None:
            _steer_angle = best_angle
        else:
            best_idx = max(range(len(sector_dists)),
                          key=lambda i: sector_dists[i])
            _steer_angle = sector_angles[best_idx]

        speed = SLOW_SPEED if front_dist < DEPTH_SLOW_DIST else DRIVE_SPEED
        return _steer_toward(rh, _steer_angle, speed)

    return (0, 0)


def _trigger_backup(heading, step):
    global _state, _backup_timer, _backup_heading, _stall_counter
    _state = "BACKUP"
    _backup_timer = 0
    _backup_heading = heading
    _stall_counter = 0


def _choose_avoidance_direction(current_heading):
    """Check left vs right of obstacle in VFH. Returns +1, -1, or None."""
    sector_angles, sector_dists = lidar_processing.get_histogram()
    if not sector_angles:
        return 1

    n = len(sector_angles)
    rx, ry, _ = odometry.get_pose()
    goal_angle = math.atan2(_goal_y - ry, _goal_x - rx)

    obs_idx = 0
    best_err = float('inf')
    for i, a in enumerate(sector_angles):
        err = abs(_angle_diff(a, _backup_heading))
        if err < best_err:
            best_err = err
            obs_idx = i

    arc_size = n // 4

    left_min = float('inf')
    for offset in range(1, arc_size + 1):
        idx = (obs_idx + offset) % n
        if sector_dists[idx] < left_min:
            left_min = sector_dists[idx]

    right_min = float('inf')
    for offset in range(1, arc_size + 1):
        idx = (obs_idx - offset) % n
        if sector_dists[idx] < right_min:
            right_min = sector_dists[idx]

    left_ok = left_min >= MIN_SIDE_CLEARANCE
    right_ok = right_min >= MIN_SIDE_CLEARANCE

    if left_ok and right_ok:
        left_angle = sector_angles[(obs_idx + arc_size // 2) % n]
        right_angle = sector_angles[(obs_idx - arc_size // 2) % n]
        left_err = abs(_angle_diff(left_angle, goal_angle))
        right_err = abs(_angle_diff(right_angle, goal_angle))
        choice = 1 if left_err <= right_err else -1
        print(f"[NAV] Both clear (L={left_min:.1f}m R={right_min:.1f}m), "
              f"picking {'LEFT' if choice > 0 else 'RIGHT'}")
        return choice
    elif left_ok:
        print(f"[NAV] Left clear ({left_min:.1f}m), right blocked ({right_min:.1f}m)")
        return 1
    elif right_ok:
        print(f"[NAV] Right clear ({right_min:.1f}m), left blocked ({left_min:.1f}m)")
        return -1
    else:
        print(f"[NAV] Both blocked (L={left_min:.1f}m R={right_min:.1f}m)")
        return None


def get_path():
    rx, ry, _ = odometry.get_pose()
    end_x = rx + 1.0 * math.cos(_steer_angle)
    end_y = ry + 1.0 * math.sin(_steer_angle)
    return [(rx, ry), (end_x, end_y)]


def get_status():
    rx, ry, _ = odometry.get_pose()
    dist = math.sqrt((_goal_x - rx)**2 + (_goal_y - ry)**2)
    front = min(_depth_front, lidar_processing.get_front_distance())
    return f"{_state:10s} | Goal: {dist:.2f}m | Front: {front:.2f}m"


# VFH gap selection

def _find_best_gap(angles, dists, goal_angle):
    n = len(dists)
    threshold = lidar_processing.OBSTACLE_DIST
    is_open = [d >= threshold for d in dists]

    gaps = []
    i = 0
    while i < n:
        if is_open[i]:
            start = i
            while i < n and is_open[i]:
                i += 1
            end = i - 1
            if (end - start + 1) >= 2:
                gaps.append((start, end))
        else:
            i += 1

    if not gaps:
        return None

    best_gap = None
    best_err = float('inf')
    for start, end in gaps:
        mid = (start + end) // 2
        err = abs(_angle_diff(angles[mid], goal_angle))
        if err < best_err:
            best_err = err
            best_gap = (start, end)

    if best_gap is None:
        return None

    start, end = best_gap
    best_sector = start
    best_sector_err = float('inf')
    for s in range(start, end + 1):
        err = abs(_angle_diff(angles[s], goal_angle))
        if err < best_sector_err:
            best_sector_err = err
            best_sector = s

    return angles[best_sector]


def _angle_diff(a, b):
    d = a - b
    while d > math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def _steer_toward(current_heading, target_angle, speed):
    err = _angle_diff(target_angle, current_heading)
    abs_err = abs(err)

    # Large error: turn in place, no forward motion
    if abs_err > math.radians(30):
        d = 1 if err > 0 else -1
        return (-TURN_SPEED * d * 0.7, TURN_SPEED * d * 0.7)

    # Medium error (15-30 deg): slow forward with strong turn
    if abs_err > math.radians(15):
        turn_cmd = max(-TURN_SPEED, min(TURN_SPEED, err * 2.0))
        fwd = SLOW_SPEED * 0.5
        return (fwd - turn_cmd * 0.5, fwd + turn_cmd * 0.5)

    # Small error (<15 deg): normal speed with gentle steering
    turn_cmd = max(-TURN_SPEED, min(TURN_SPEED, err * 1.5))
    return (speed - turn_cmd * 0.3, speed + turn_cmd * 0.3)
