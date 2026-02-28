import math
import csv
import os

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("[WARNING] numpy not available - using slower pure-Python mode")

MATPLOTLIB_AVAILABLE = False
plt = None

for _backend in ['Qt5Agg', 'TkAgg', 'wxAgg', 'Agg']:
    try:
        import matplotlib
        matplotlib.use(_backend)
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors
        _test_fig = plt.figure()
        plt.close(_test_fig)
        MATPLOTLIB_AVAILABLE = True
        print(f"[OK] matplotlib backend: {_backend}")
        break
    except Exception:
        pass

if not MATPLOTLIB_AVAILABLE:
    print("[WARNING] matplotlib not available")


# Arena dimensions (meters)
ARENA_WIDTH  = 7.18
ARENA_LENGTH = 4.57

GRID_RESOLUTION = 0.05  # meters per cell

# Log-odds update values
LOG_ODDS_HIT     =  0.7
LOG_ODDS_MISS    = -0.2
LOG_ODDS_MIN     = -2.0
LOG_ODDS_MAX     =  5.0
LOG_ODDS_UNKNOWN =  0.0

FREE_SPACE_RAY_MAX = 1.5  # max distance to mark free space (meters)

# Rangefinder parameters
RANGEFINDER_HFOV      = 90.0
RANGE_MAX             = 5.0
RANGE_MIN             = 0.15
RANGE_DEPTH_STEP      = 4
HEIGHT_ROW_START_FRAC = 0.35
HEIGHT_ROW_END_FRAC   = 0.80

# Robot physical dimensions
WHEEL_RADIUS = 0.15
WHEEL_BASE   = 0.5


class OccupancyGrid:
    """
    2D log-odds occupancy grid.
    log-odds > 0 = obstacle, = 0 = unknown, < 0 = free.
    Cost scale: 0 (free) to 100 (obstacle).
    """

    def __init__(self, width_m, height_m, resolution, origin_x=0.0, origin_y=0.0):
        self.resolution = resolution
        self.origin_x   = origin_x
        self.origin_y   = origin_y
        self.cols = int(math.ceil(width_m  / resolution))
        self.rows = int(math.ceil(height_m / resolution))

        if NUMPY_AVAILABLE:
            self.log_odds = np.full((self.rows, self.cols), LOG_ODDS_UNKNOWN, dtype=np.float32)
        else:
            self.log_odds = [[LOG_ODDS_UNKNOWN] * self.cols for _ in range(self.rows)]

        print(f"[MAP] Grid: {self.cols}x{self.rows} cells ({self.cols*resolution:.2f}m x {self.rows*resolution:.2f}m)")

    def world_to_cell(self, wx, wy):
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return col, row

    def cell_to_world(self, col, row):
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, col, row):
        return 0 <= col < self.cols and 0 <= row < self.rows

    def _get(self, col, row):
        if NUMPY_AVAILABLE:
            return float(self.log_odds[row, col])
        return self.log_odds[row][col]

    def _set(self, col, row, val):
        val = max(LOG_ODDS_MIN, min(LOG_ODDS_MAX, val))
        if NUMPY_AVAILABLE:
            self.log_odds[row, col] = val
        else:
            self.log_odds[row][col] = val

    def get_log_odds(self, col, row):
        if self.in_bounds(col, row):
            return self._get(col, row)
        return LOG_ODDS_MAX  # out of bounds treated as obstacle

    def mark_obstacle(self, col, row):
        if self.in_bounds(col, row):
            self._set(col, row, self._get(col, row) + LOG_ODDS_HIT)

    def mark_free(self, col, row):
        if self.in_bounds(col, row):
            self._set(col, row, self._get(col, row) + LOG_ODDS_MISS)

    def cast_ray(self, robot_col, robot_row, hit_col, hit_row, is_obstacle):
        """Mark intermediate cells free, endpoint as obstacle if is_obstacle=True."""
        cells = self._bresenham(robot_col, robot_row, hit_col, hit_row)
        for i, (c, r) in enumerate(cells):
            if not self.in_bounds(c, r):
                break
            if i == len(cells) - 1:
                if is_obstacle:
                    self.mark_obstacle(c, r)
            else:
                self.mark_free(c, r)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x  += sx
            if e2 < dx:
                err += dx
                y  += sy
            if len(cells) > 1000:
                break
        return cells

    def log_odds_to_cost(self, lo):
        prob = 1.0 - 1.0 / (1.0 + math.exp(lo))
        return int(prob * 100)

    def get_cost_grid(self):
        if NUMPY_AVAILABLE:
            prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
            return (prob * 100).astype(np.int32)
        return [[self.log_odds_to_cost(self.log_odds[r][c])
                 for c in range(self.cols)] for r in range(self.rows)]

    def get_cost_at(self, col, row):
        if not self.in_bounds(col, row):
            return 100
        return self.log_odds_to_cost(self._get(col, row))

    def save_csv(self, filepath, robot_col=None, robot_row=None):
        cost_grid = self.get_cost_grid()
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Y\\X (m)'] + [f'{self.origin_x + (c+0.5)*self.resolution:.2f}'
                                             for c in range(self.cols)])
            for r in range(self.rows - 1, -1, -1):
                wy = self.origin_y + (r + 0.5) * self.resolution
                row_data = cost_grid[r, :].tolist() if NUMPY_AVAILABLE else cost_grid[r]
                if (robot_col is not None and robot_row is not None
                        and 0 <= robot_col < self.cols and robot_row == r):
                    row_data = list(row_data)
                    row_data[robot_col] = -1
                writer.writerow([f'{wy:.2f}'] + row_data)
        print(f"[MAP] Saved → {filepath}")

    def get_display_array(self, robot_col=None, robot_row=None):
        if not NUMPY_AVAILABLE:
            return None
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        display = prob.copy()
        if robot_col is not None and robot_row is not None:
            if 0 <= robot_col < self.cols and 0 <= robot_row < self.rows:
                display[robot_row, robot_col] = 0.5
        return display


class DeadReckoning:
    """Wheel encoder + IMU dead reckoning. IMU provides heading, encoders provide distance."""

    def __init__(self, wheel_radius=WHEEL_RADIUS, wheel_base=WHEEL_BASE):
        self.wheel_radius = wheel_radius
        self.wheel_base   = wheel_base
        self.x = self.y = self.heading = 0.0
        self.prev_FL = self.prev_FR = self.prev_RL = self.prev_RR = None

    def set_position(self, x, y, heading=None):
        self.x, self.y = x, y
        if heading is not None:
            self.heading = heading

    def initialize_encoders(self, pos_FL, pos_FR, pos_RL, pos_RR):
        self.prev_FL, self.prev_FR = pos_FL, pos_FR
        self.prev_RL, self.prev_RR = pos_RL, pos_RR

    def update(self, pos_FL, pos_FR, pos_RL, pos_RR, yaw_from_imu, dt):
        """Returns (x, y, heading, v_forward)."""
        if self.prev_FL is None:
            self.initialize_encoders(pos_FL, pos_FR, pos_RL, pos_RR)
            return self.x, self.y, self.heading, 0.0

        omega_left  = ((pos_FL - self.prev_FL) + (pos_RL - self.prev_RL)) / (2.0 * dt)
        omega_right = ((pos_FR - self.prev_FR) + (pos_RR - self.prev_RR)) / (2.0 * dt)

        v_forward = (omega_left + omega_right) / 2.0 * self.wheel_radius
        self.heading = yaw_from_imu  # use IMU directly, avoids integrating drift

        self.x += v_forward * math.cos(self.heading) * dt
        self.y += v_forward * math.sin(self.heading) * dt

        self.prev_FL, self.prev_FR = pos_FL, pos_FR
        self.prev_RL, self.prev_RR = pos_RL, pos_RR

        return self.x, self.y, self.heading, v_forward


def depth_image_to_obstacles(depth_data, rf_width, rf_height,
                               robot_x, robot_y, robot_heading,
                               hfov_deg=RANGEFINDER_HFOV,
                               col_step=RANGE_DEPTH_STEP):
    """Convert rangefinder depth image to (world_x, world_y, is_obstacle) tuples."""
    if depth_data is None:
        return []

    results = []
    hfov_rad = math.radians(hfov_deg)
    row_start = int(rf_height * HEIGHT_ROW_START_FRAC)
    row_end   = int(rf_height * HEIGHT_ROW_END_FRAC)
    if row_start >= row_end:
        row_start, row_end = 0, rf_height

    if NUMPY_AVAILABLE:
        if isinstance(depth_data, (bytes, bytearray)):
            depth_arr = np.frombuffer(bytes(depth_data), dtype=np.float32)
        elif isinstance(depth_data, list):
            depth_arr = np.array(depth_data, dtype=np.float32)
        else:
            depth_arr = np.asarray(depth_data, dtype=np.float32)

        if depth_arr.size != rf_height * rf_width:
            return []
        depth_arr = depth_arr.reshape((rf_height, rf_width))

        for col in range(0, rf_width, col_step):
            h_angle  = (col / rf_width - 0.5) * hfov_rad
            col_band = depth_arr[row_start:row_end, col]
            mask     = np.isfinite(col_band) & (col_band > RANGE_MIN) & (col_band < RANGE_MAX)
            valid    = col_band[mask]

            if len(valid) == 0:
                d, is_obstacle = FREE_SPACE_RAY_MAX, False
            else:
                d, is_obstacle = float(np.min(valid)), True

            local_fwd   = d * math.cos(h_angle)
            local_right = d * math.sin(h_angle)
            wx = robot_x + local_fwd * math.cos(robot_heading) - local_right * math.sin(robot_heading)
            wy = robot_y + local_fwd * math.sin(robot_heading) + local_right * math.cos(robot_heading)
            results.append((wx, wy, is_obstacle))

    else:
        if isinstance(depth_data, (bytes, bytearray)):
            import struct
            n = len(depth_data) // 4
            flat = list(struct.unpack(f'{n}f', depth_data))
        else:
            flat = list(depth_data)

        def get_px(r, c):
            idx = r * rf_width + c
            if idx < len(flat):
                v = flat[idx]
                if math.isfinite(v) and RANGE_MIN < v < RANGE_MAX:
                    return v
            return None

        for col in range(0, rf_width, col_step):
            h_angle = (col / rf_width - 0.5) * hfov_rad
            depths  = [v for v in (get_px(r, col) for r in range(row_start, row_end)) if v is not None]

            if not depths:
                d, is_obstacle = FREE_SPACE_RAY_MAX, False
            else:
                d, is_obstacle = min(depths), True

            local_fwd   = d * math.cos(h_angle)
            local_right = d * math.sin(h_angle)
            wx = robot_x + local_fwd * math.cos(robot_heading) - local_right * math.sin(robot_heading)
            wy = robot_y + local_fwd * math.sin(robot_heading) + local_right * math.cos(robot_heading)
            results.append((wx, wy, is_obstacle))

    return results


def get_front_obstacle_distance(depth_data, rf_width, rf_height):
    """Returns distance to nearest obstacle directly ahead, or inf if none."""
    if depth_data is None or not NUMPY_AVAILABLE:
        return float('inf')

    if isinstance(depth_data, (bytes, bytearray)):
        depth_arr = np.frombuffer(bytes(depth_data), dtype=np.float32)
    elif isinstance(depth_data, list):
        depth_arr = np.array(depth_data, dtype=np.float32)
    else:
        depth_arr = np.asarray(depth_data, dtype=np.float32)

    if depth_arr.size != rf_height * rf_width:
        return float('inf')

    depth_arr = depth_arr.reshape((rf_height, rf_width))
    row_start = int(rf_height * HEIGHT_ROW_START_FRAC)
    row_end   = int(rf_height * HEIGHT_ROW_END_FRAC)
    center    = depth_arr[row_start:row_end, int(rf_width*0.4):int(rf_width*0.6)]
    valid     = center[(center > RANGE_MIN) & (center < RANGE_MAX) & np.isfinite(center)]

    return float(np.min(valid)) if len(valid) > 0 else float('inf')


class MapVisualizer:
    """Real-time matplotlib occupancy grid display."""

    CMAP = None

    def __init__(self, grid, script_dir="."):
        self.grid           = grid
        self.fig = self.ax = self.im = None
        self.robot_dot      = None
        self.robot_arrow    = None
        self.trajectory_line = None
        self.trajectory_x   = []
        self.trajectory_y   = []
        self._agg_only      = False
        self._snapshot_path = os.path.join(script_dir, "map_snapshot.png")
        self.robot_heading  = 0.0
        self.path_line = self.waypoint_dots = self.goal_dot = None
        self.goal_x = self.goal_y = self.path_world = None

        if MATPLOTLIB_AVAILABLE:
            try:
                self._init_figure()
            except Exception as e:
                print(f"[WARNING] Visualization init failed: {e}")

    def _init_figure(self):
        import matplotlib.pyplot as _plt
        import matplotlib.colors as _mc

        MapVisualizer.CMAP = _mc.LinearSegmentedColormap.from_list(
            'occ_grid',
            [(0.0, (0.95, 0.95, 0.95)), (0.45, (1.0, 1.0, 1.0)),
             (0.55, (1.0, 1.0, 1.0)),  (1.0, (0.1, 0.1, 0.1))],
            N=256
        )

        _plt.ion()
        self.fig, self.ax = _plt.subplots(figsize=(12, 8))
        try:
            self.fig.canvas.manager.set_window_title('Lunabotics Map')
        except AttributeError:
            pass

        data = self.grid.get_display_array()
        if data is None:
            return

        extent = [self.grid.origin_x,
                  self.grid.origin_x + self.grid.cols * self.grid.resolution,
                  self.grid.origin_y,
                  self.grid.origin_y + self.grid.rows * self.grid.resolution]

        self.im = self.ax.imshow(data, origin='lower', extent=extent,
                                  cmap=self.CMAP, vmin=0.0, vmax=1.0,
                                  interpolation='nearest', aspect='equal')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Occupancy Grid')

        self.robot_dot,       = self.ax.plot([], [], 'bo', markersize=10, label='Robot', zorder=10)
        self.robot_arrow      = self.ax.annotate('', xy=(0,0), xytext=(0,0),
                                                  arrowprops=dict(arrowstyle='->', color='blue', lw=2), zorder=11)
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.3, label='Trajectory', zorder=5)
        self.path_line,       = self.ax.plot([], [], 'g-', linewidth=2, alpha=0.7, label='Path', zorder=6)
        self.waypoint_dots,   = self.ax.plot([], [], 'go', markersize=5, alpha=0.5, zorder=7)
        self.goal_dot,        = self.ax.plot([], [], 'g*', markersize=15, label='Goal', zorder=9)

        _plt.colorbar(self.im, ax=self.ax, label='Obstacle probability')
        self.ax.legend(loc='upper right')
        _plt.tight_layout()
        self.fig.canvas.draw_idle()
        try:
            self.fig.canvas.flush_events()
        except Exception:
            pass

        backend_name = matplotlib.get_backend().lower()
        self._agg_only = ('agg' in backend_name and 'qt' not in backend_name
                          and 'tk' not in backend_name and 'wx' not in backend_name)

    def set_goal(self, goal_x, goal_y):
        self.goal_x, self.goal_y = goal_x, goal_y

    def set_path(self, path_world):
        self.path_world = path_world

    def set_heading(self, heading):
        self.robot_heading = heading

    def set_title(self, title):
        if self.ax:
            self.ax.set_title(title)

    def update(self, robot_x, robot_y):
        if not MATPLOTLIB_AVAILABLE or self.im is None:
            return
        try:
            robot_col, robot_row = self.grid.world_to_cell(robot_x, robot_y)
            data = self.grid.get_display_array(robot_col, robot_row)
            if data is None:
                return

            self.im.set_data(data)
            self.robot_dot.set_data([robot_x], [robot_y])

            arrow_len = 0.3
            self.robot_arrow.set_position((robot_x, robot_y))
            self.robot_arrow.xy = (robot_x + arrow_len * math.cos(self.robot_heading),
                                   robot_y + arrow_len * math.sin(self.robot_heading))

            self.trajectory_x.append(robot_x)
            self.trajectory_y.append(robot_y)
            self.trajectory_line.set_data(self.trajectory_x, self.trajectory_y)

            if self.goal_x is not None:
                self.goal_dot.set_data([self.goal_x], [self.goal_y])

            if self.path_world:
                self.path_line.set_data([p[0] for p in self.path_world],
                                         [p[1] for p in self.path_world])
                self.waypoint_dots.set_data([p[0] for p in self.path_world],
                                             [p[1] for p in self.path_world])
            else:
                self.path_line.set_data([], [])
                self.waypoint_dots.set_data([], [])

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

            if self._agg_only:
                self.fig.savefig(self._snapshot_path, dpi=90, bbox_inches='tight')

        except Exception as e:
            print(f"[VIZ] Error: {e}")

    def close(self):
        if MATPLOTLIB_AVAILABLE and self.fig:
            try:
                import matplotlib.pyplot as _plt
                if not self._agg_only:
                    _plt.ioff()
                self.fig.savefig(self._snapshot_path, dpi=120, bbox_inches='tight')
                print(f"[VIZ] Final image → {self._snapshot_path}")
                _plt.close(self.fig)
            except Exception as e:
                print(f"[VIZ] Close error: {e}")