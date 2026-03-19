import math
import odometry

# LiDAR config
LIDAR_FOV        = 6.28     # RPLidarA2 full rotation (radians)
SENSOR_HEIGHT    = 0.50     # mount height above ground (m)
GROUND_THRESHOLD = 0.08     # world Z below this is a floor hit

# Range limits
MAX_RANGE = 8.0
MIN_RANGE = 0.50            # reject body self-returns

# VFH histogram
NUM_SECTORS   = 72          # 5 degrees each across 360
OBSTACLE_DIST = 2.5         # sector blocked if closest point < this

# Clustering (DBSCAN-style)
CLUSTER_EPS     = 0.25
CLUSTER_MIN_PTS = 3


WALL_ASPECT_RATIO   = 3.0
MAX_ROCK_DIAMETER   = 0.8
MIN_ROCK_DIAMETER   = 0.05
LINEARITY_THRESHOLD = 4.0
MAX_ROCK_POINTS     = 40

LANDMARK_MATCH_DIST = 0.6
LANDMARK_MERGE_DIST = 0.4
MATCH_MIN_LANDMARKS = 2
CORRECTION_GAIN     = 0.3

# Front obstacle detection cone
FRONT_CONE_HALF_ANGLE = math.radians(20)
FRONT_CONE_MAX_DIST   = 3.0

# Per-frame state
_lidar = None
_last_front_dist = float('inf')
_landmarks = []

_sector_angles = []
_sector_dists  = []
_scan_world    = []
_obstacle_pts  = []


def setup(lidar_device):
    global _lidar
    _lidar = lidar_device
    try:
        _lidar.enablePointCloud()
    except Exception:
        pass
    print(f"[LIDAR] VFH mode, {NUM_SECTORS} sectors, "
          f"range [{MIN_RANGE:.2f}, {MAX_RANGE:.1f}]m")


def update(cloud):
    global _last_front_dist, _sector_angles, _sector_dists
    global _scan_world, _obstacle_pts

    if cloud is None or len(cloud) == 0:
        return

    rx, ry, rh = odometry.get_pose()
    cos_h = math.cos(rh)
    sin_h = math.sin(rh)

    scan_world = []
    obstacle_pts = []
    front_min = float('inf')
    sector_min = [float('inf')] * NUM_SECTORS

    for pt in cloud:
        lx, ly, lz = _read_point(pt)

        if (math.isinf(lx) or math.isinf(ly) or math.isinf(lz)
                or math.isnan(lx) or math.isnan(ly) or math.isnan(lz)):
            continue

        horiz = math.sqrt(lx * lx + ly * ly)
        if horiz < MIN_RANGE or horiz > MAX_RANGE:
            continue

        world_z = SENSOR_HEIGHT + lz
        is_ground = world_z < GROUND_THRESHOLD

        wx = rx + lx * cos_h - ly * sin_h
        wy = ry + lx * sin_h + ly * cos_h
        scan_world.append((wx, wy, is_ground))

        if is_ground:
            continue

        obstacle_pts.append((wx, wy))

        # front distance for emergency stop
        fwd_angle = math.atan2(ly, lx)
        if abs(fwd_angle) <= FRONT_CONE_HALF_ANGLE and horiz <= FRONT_CONE_MAX_DIST:
            front_min = min(front_min, horiz)

        # VFH sector assignment
        local_angle = math.atan2(ly, lx)
        frac = (local_angle + math.pi) / (2.0 * math.pi)
        sector = int(frac * NUM_SECTORS)
        sector = max(0, min(NUM_SECTORS - 1, sector))
        if horiz < sector_min[sector]:
            sector_min[sector] = horiz

    # sector angles in world frame
    _sector_angles = []
    for i in range(NUM_SECTORS):
        local_a = -math.pi + (i + 0.5) / NUM_SECTORS * 2.0 * math.pi
        _sector_angles.append(rh + local_a)
    _sector_dists = sector_min
    _scan_world = scan_world
    _obstacle_pts = obstacle_pts
    _last_front_dist = front_min

    # cluster obstacle points, classify, only rocks feed landmark DB
    if len(obstacle_pts) >= CLUSTER_MIN_PTS:
        clusters = _cluster_points(obstacle_pts)
        rock_observations = []
        for indices in clusters:
            pts = [obstacle_pts[i] for i in indices]
            cx, cy, w, h, aspect = _cluster_geometry(pts)
            diameter = max(w, h)
            linearity = _compute_linearity(pts)
            is_rock = (aspect < WALL_ASPECT_RATIO
                       and diameter < MAX_ROCK_DIAMETER
                       and diameter > MIN_ROCK_DIAMETER
                       and linearity < LINEARITY_THRESHOLD
                       and len(pts) < MAX_ROCK_POINTS)
            if is_rock:
                rock_observations.append((cx, cy))
        if rock_observations:
            _update_landmarks_and_correct(rock_observations)


def get_histogram():
    return (_sector_angles, _sector_dists)

def get_scan_world():
    return _scan_world

def get_obstacle_points():
    return _obstacle_pts

def get_landmarks():
    return _landmarks

def get_front_distance():
    return _last_front_dist


# -- point reading --

def _read_point(pt):
    if hasattr(pt, 'x'):
        return pt.x, pt.y, pt.z
    return pt[0], pt[1], pt[2]


# -- clustering --

def _cluster_points(pts):
    n = len(pts)
    used = [False] * n
    eps_sq = CLUSTER_EPS * CLUSTER_EPS
    clusters = []
    for i in range(n):
        if used[i]:
            continue
        cluster = [i]
        used[i] = True
        queue = [i]
        while queue:
            ci = queue.pop()
            px, py = pts[ci]
            for j in range(n):
                if used[j]:
                    continue
                dx = pts[j][0] - px
                dy = pts[j][1] - py
                if dx * dx + dy * dy <= eps_sq:
                    used[j] = True
                    cluster.append(j)
                    queue.append(j)
        if len(cluster) >= CLUSTER_MIN_PTS:
            clusters.append(cluster)
    return clusters


def _cluster_geometry(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    short = min(w, h) if min(w, h) > 0.01 else 0.01
    aspect = max(w, h) / short
    return cx, cy, w, h, aspect


def _compute_linearity(pts):
    """Eigenvalue ratio of 2D point spread. High = linear, low = blobby."""
    n = len(pts)
    if n < 3:
        return 1.0
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    cxx = sum((p[0] - cx)**2 for p in pts) / n
    cyy = sum((p[1] - cy)**2 for p in pts) / n
    cxy = sum((p[0] - cx) * (p[1] - cy) for p in pts) / n
    trace = cxx + cyy
    det = cxx * cyy - cxy * cxy
    disc = max(0.0, trace * trace / 4.0 - det)
    sqrt_disc = math.sqrt(disc)
    e1 = trace / 2.0 + sqrt_disc
    e2 = trace / 2.0 - sqrt_disc
    if e2 < 1e-8:
        return 100.0
    return e1 / e2

def _update_landmarks_and_correct(rock_observations):
    global _landmarks

    matches = []
    unmatched = []

    for obs_x, obs_y in rock_observations:
        best_dist = float('inf')
        best_idx = -1
        for idx, lm in enumerate(_landmarks):
            d = math.sqrt((obs_x - lm['x'])**2 + (obs_y - lm['y'])**2)
            if d < best_dist:
                best_dist = d
                best_idx = idx

        if best_dist < LANDMARK_MATCH_DIST and best_idx >= 0:
            lm = _landmarks[best_idx]
            matches.append((obs_x, obs_y, lm['x'], lm['y']))
            n = lm['obs_count']
            lm['x'] = (lm['x'] * n + obs_x) / (n + 1)
            lm['y'] = (lm['y'] * n + obs_y) / (n + 1)
            lm['obs_count'] = n + 1
        else:
            unmatched.append((obs_x, obs_y))

    for ox, oy in unmatched:
        too_close = False
        for lm in _landmarks:
            if math.sqrt((ox - lm['x'])**2 + (oy - lm['y'])**2) < LANDMARK_MERGE_DIST:
                too_close = True
                break
        if not too_close:
            _landmarks.append({'x': ox, 'y': oy, 'obs_count': 1})

    if len(matches) >= MATCH_MIN_LANDMARKS:
        dx_sum = sum(lm_x - obs_x for obs_x, _, lm_x, _ in matches)
        dy_sum = sum(lm_y - obs_y for _, obs_y, _, lm_y in matches)
        n = len(matches)
        odometry.apply_correction(
            (dx_sum / n) * CORRECTION_GAIN,
            (dy_sum / n) * CORRECTION_GAIN)