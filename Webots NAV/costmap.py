import math

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

import lidar_processing

# Grid parameters
RESOLUTION = 0.10           # meters per cell
GRID_WIDTH = 100            # cells (10m at 0.10 resolution)
GRID_HEIGHT = 100           # cells (10m at 0.10 resolution)
ORIGIN_X = -2.0             # world X of grid cell (0,0)
ORIGIN_Y = -2.0             # world Y of grid cell (0,0)

# Inflation radius (should be >= robot half-width + generous buffer)
INFLATE_RADIUS = 0.70       # meters, wide berth around all obstacles
INFLATE_CELLS = int(math.ceil(INFLATE_RADIUS / RESOLUTION))

# Cost values
COST_FREE     = 0
COST_OCCUPIED = 100
COST_INFLATED = 60          # cells near obstacles, higher so A* avoids them harder
COST_UNKNOWN  = 0           # treat unknown as free for navigation

# Decay: how quickly old obstacles fade. Each frame, all occupied cells
# lose this much cost. This prevents stale ghost obstacles from
# persisting when the robot moves and the LiDAR no longer sees them.
DECAY_RATE = 15             # cost units per frame

# The grid itself
_grid = None
_inflate_kernel = None


def setup():
    """Initialize the costmap grid and precompute the inflation kernel."""
    global _grid, _inflate_kernel

    if not NUMPY_AVAILABLE:
        print("[COSTMAP] numpy not available, costmap disabled")
        return

    _grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=np.float32)

    # Precompute circular inflation kernel (which cells to inflate around an obstacle)
    _inflate_kernel = []
    for dy in range(-INFLATE_CELLS, INFLATE_CELLS + 1):
        for dx in range(-INFLATE_CELLS, INFLATE_CELLS + 1):
            dist = math.sqrt(dx * dx + dy * dy) * RESOLUTION
            if dist <= INFLATE_RADIUS:
                # Cost falls off with distance from obstacle center
                cost = COST_INFLATED * (1.0 - dist / INFLATE_RADIUS)
                _inflate_kernel.append((dx, dy, cost))

    print(f"[COSTMAP] Grid {GRID_WIDTH}x{GRID_HEIGHT} at {RESOLUTION}m/cell, "
          f"inflate={INFLATE_RADIUS}m ({INFLATE_CELLS} cells)")


def update():
    """Stamp current LiDAR obstacle points into the grid and inflate."""
    global _grid

    if _grid is None:
        return

    # Decay all cells toward zero (clears stale obstacles)
    _grid = np.maximum(0, _grid - DECAY_RATE)

    # Get obstacle-height points from LiDAR (already in world coords)
    obs_pts = lidar_processing.get_obstacle_points()

    for wx, wy in obs_pts:
        gx, gy = _world_to_grid(wx, wy)
        if 0 <= gx < GRID_WIDTH and 0 <= gy < GRID_HEIGHT:
            # Mark the cell as occupied
            _grid[gy, gx] = COST_OCCUPIED

            # Inflate surrounding cells
            for dx, dy, cost in _inflate_kernel:
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT:
                    _grid[ny, nx] = max(_grid[ny, nx], cost)


def get_cost_at_world(wx, wy):
    """Look up the cost at a world coordinate. Returns 0-100."""
    if _grid is None:
        return COST_FREE
    gx, gy = _world_to_grid(wx, wy)
    if 0 <= gx < GRID_WIDTH and 0 <= gy < GRID_HEIGHT:
        return float(_grid[gy, gx])
    return COST_FREE  # outside grid = assume free


def is_collision(wx, wy):
    """Check if a world coordinate is inside an occupied or inflated cell."""
    return get_cost_at_world(wx, wy) >= COST_INFLATED * 0.5


def check_trajectory(trajectory):
    """Check a list of (x, y, ...) points against the costmap.
    Returns the minimum clearance cost (0 = fully clear, 100 = collision).
    Returns COST_OCCUPIED if any point collides."""
    if _grid is None:
        return COST_FREE

    max_cost = COST_FREE
    for point in trajectory:
        wx, wy = point[0], point[1]
        cost = get_cost_at_world(wx, wy)
        if cost >= COST_OCCUPIED:
            return COST_OCCUPIED  # definite collision
        if cost > max_cost:
            max_cost = cost
    return max_cost


def get_grid():
    """Return the raw grid for visualization. Shape: (GRID_HEIGHT, GRID_WIDTH)."""
    return _grid


def get_grid_params():
    """Return grid parameters for coordinate conversion."""
    return {
        'resolution': RESOLUTION,
        'width': GRID_WIDTH,
        'height': GRID_HEIGHT,
        'origin_x': ORIGIN_X,
        'origin_y': ORIGIN_Y,
    }


# Coordinate conversion helpers

def _world_to_grid(wx, wy):
    """Convert world coordinates to grid cell indices."""
    gx = int((wx - ORIGIN_X) / RESOLUTION)
    gy = int((wy - ORIGIN_Y) / RESOLUTION)
    return gx, gy


def _grid_to_world(gx, gy):
    """Convert grid cell indices to world coordinates (cell center)."""
    wx = ORIGIN_X + (gx + 0.5) * RESOLUTION
    wy = ORIGIN_Y + (gy + 0.5) * RESOLUTION
    return wx, wy
