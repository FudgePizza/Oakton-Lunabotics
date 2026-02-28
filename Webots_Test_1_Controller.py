from controller import Robot
import math
import os
import heapq

from mapping_module import (
    OccupancyGrid, DeadReckoning, MapVisualizer,
    depth_image_to_obstacles, get_front_obstacle_distance,
    ARENA_WIDTH, ARENA_LENGTH, GRID_RESOLUTION,
    NUMPY_AVAILABLE, MATPLOTLIB_AVAILABLE
)

if NUMPY_AVAILABLE:
    import numpy as np


# Goal and start positions
GOAL_X = 5.0
GOAL_Y = 2.0
GOAL_TOLERANCE = 0.20
ROBOT_START_X  = 0.6
ROBOT_START_Y  = 0.8

# Motor speeds (rad/s)
DRIVE_SPEED = 2.0
TURN_SPEED  = 1.5
SLOW_SPEED  = 1.0

# Path following tolerances
WAYPOINT_TOLERANCE = 0.25  # meters
HEADING_TOLERANCE  = 0.15  # radians

# Obstacle distances (meters)
OBSTACLE_STOP_DIST = 0.4
OBSTACLE_SLOW_DIST = 0.8

# Cells with cost >= this are treated as obstacles
OBSTACLE_COST_THRESHOLD = 75
PATH_SIMPLIFY_SKIP = 5

SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
MAP_LIVE_PATH  = os.path.join(SCRIPT_DIR, "map_live.csv")
MAP_FINAL_PATH = os.path.join(SCRIPT_DIR, "map_final.csv")

VIZ_UPDATE_INTERVAL = 15
MAP_SAVE_INTERVAL   = 150
SHOW_VISUALIZATION  = True


class AStarPlanner:
    """A* on occupancy grid. Unknown cells (~50 cost) are treated as passable."""

    DIRECTIONS = [
        ( 0,  1, 1.0), ( 0, -1, 1.0), ( 1,  0, 1.0), (-1,  0, 1.0),
        ( 1,  1, 1.414), (-1,  1, 1.414), ( 1, -1, 1.414), (-1, -1, 1.414),
    ]

    def __init__(self, grid, obstacle_threshold=OBSTACLE_COST_THRESHOLD):
        self.grid = grid
        self.obstacle_threshold = obstacle_threshold

    def heuristic(self, col1, row1, col2, row2):
        return math.sqrt((col2 - col1)**2 + (row2 - row1)**2)

    def is_passable(self, col, row):
        if not self.grid.in_bounds(col, row):
            return False
        return self.grid.get_cost_at(col, row) < self.obstacle_threshold

    def find_path(self, start_col, start_row, goal_col, goal_row):
        """Returns list of (col, row) waypoints, or empty list if no path found."""
        if not self.grid.in_bounds(start_col, start_row):
            print("[A*] Start out of bounds!")
            return []
        if not self.grid.in_bounds(goal_col, goal_row):
            print("[A*] Goal out of bounds!")
            return []

        if not self.is_passable(goal_col, goal_row):
            goal_col, goal_row = self._find_nearest_passable(goal_col, goal_row)
            if goal_col is None:
                print("[A*] No passable cell near goal!")
                return []

        open_set = []
        counter  = 0
        heapq.heappush(open_set, (self.heuristic(start_col, start_row, goal_col, goal_row),
                                   counter, start_col, start_row))
        counter += 1

        came_from   = {}
        g_cost      = {(start_col, start_row): 0.0}
        in_open_set = {(start_col, start_row)}
        iterations  = 0
        max_iter    = self.grid.cols * self.grid.rows

        while open_set and iterations < max_iter:
            iterations += 1
            _, _, current_col, current_row = heapq.heappop(open_set)
            current = (current_col, current_row)
            in_open_set.discard(current)

            if current_col == goal_col and current_row == goal_row:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                print(f"[A*] Path found: {len(path)} points, {iterations} iterations")
                return path

            for dcol, drow, move_cost in self.DIRECTIONS:
                ncol, nrow = current_col + dcol, current_row + drow
                neighbor   = (ncol, nrow)

                if not self.is_passable(ncol, nrow):
                    continue

                cost_penalty   = (self.grid.get_cost_at(ncol, nrow) / 100.0) * 0.3
                tentative_g    = g_cost[current] + move_cost + cost_penalty

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    came_from[neighbor] = current
                    g_cost[neighbor]    = tentative_g
                    h = self.heuristic(ncol, nrow, goal_col, goal_row)
                    if neighbor not in in_open_set:
                        heapq.heappush(open_set, (tentative_g + h, counter, ncol, nrow))
                        counter += 1
                        in_open_set.add(neighbor)

        print(f"[A*] No path found after {iterations} iterations")
        return []

    def _find_nearest_passable(self, col, row, max_radius=15):
        for radius in range(1, max_radius + 1):
            for dc in range(-radius, radius + 1):
                for dr in range(-radius, radius + 1):
                    if abs(dc) == radius or abs(dr) == radius:
                        nc, nr = col + dc, row + dr
                        if self.is_passable(nc, nr):
                            return nc, nr
        return None, None

    def simplify_path(self, path, skip=PATH_SIMPLIFY_SKIP):
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        for i in range(skip, len(path) - 1, skip):
            simplified.append(path[i])
        simplified.append(path[-1])
        return simplified


class NavState:
    IDLE       = "IDLE"
    PLANNING   = "PLANNING"
    NAVIGATING = "NAVIGATING"
    BLOCKED    = "BLOCKED"
    SCANNING   = "SCANNING"
    ARRIVED    = "ARRIVED"


class Navigator:
    """
    State machine: IDLE → PLANNING → NAVIGATING → ARRIVED
    When blocked: NAVIGATING → BLOCKED → SCANNING → PLANNING → NAVIGATING
    """

    def __init__(self, grid, odometry, goal_x, goal_y):
        self.grid     = grid
        self.odometry = odometry
        self.goal_x   = goal_x
        self.goal_y   = goal_y
        self.planner  = AStarPlanner(grid)

        self.state        = NavState.IDLE
        self.path         = []
        self.path_world   = []
        self.waypoint_idx = 0

        self.blocked_timer  = 0
        self.scan_timer     = 0
        self.scan_direction = 1  # alternates between left/right each block sequence
        self.front_dist     = float('inf')
        self.replan_count   = 0

    def start(self):
        self.state = NavState.PLANNING
        print(f"[NAV] Starting → ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def stop(self):
        self.state = NavState.IDLE
        self.path = []
        self.path_world = []
        print("[NAV] Stopped")

    def distance_to_goal(self):
        dx = self.goal_x - self.odometry.x
        dy = self.goal_y - self.odometry.y
        return math.sqrt(dx*dx + dy*dy)

    def angle_to(self, target_x, target_y):
        return math.atan2(target_y - self.odometry.y, target_x - self.odometry.x)

    def angle_error(self, target_angle):
        diff = target_angle - self.odometry.heading
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def set_front_distance(self, dist):
        self.front_dist = dist

    def _plan_path(self):
        start_col, start_row = self.grid.world_to_cell(self.odometry.x, self.odometry.y)
        goal_col,  goal_row  = self.grid.world_to_cell(self.goal_x, self.goal_y)

        self.path = self.planner.find_path(start_col, start_row, goal_col, goal_row)
        if not self.path:
            return False

        self.path       = self.planner.simplify_path(self.path)
        self.path_world = [self.grid.cell_to_world(c, r) for c, r in self.path]
        self.waypoint_idx = 1 if len(self.path) > 1 else 0
        self.replan_count += 1
        return True

    def _current_waypoint(self):
        if self.waypoint_idx < len(self.path_world):
            return self.path_world[self.waypoint_idx]
        return (self.goal_x, self.goal_y)

    def _advance_waypoint(self):
        if self.waypoint_idx >= len(self.path_world):
            return False
        wx, wy = self.path_world[self.waypoint_idx]
        dist = math.sqrt((wx - self.odometry.x)**2 + (wy - self.odometry.y)**2)
        if dist < WAYPOINT_TOLERANCE:
            self.waypoint_idx += 1
            return True
        return False

    def _is_path_blocked(self):
        if not self.path:
            return False
        for i in range(self.waypoint_idx, min(self.waypoint_idx + 3, len(self.path))):
            col, row = self.path[i]
            if self.grid.get_cost_at(col, row) >= OBSTACLE_COST_THRESHOLD:
                return True
        return False

    def update(self, step):
        """Returns (left_speed, right_speed)."""
        if self.distance_to_goal() < GOAL_TOLERANCE:
            if self.state != NavState.ARRIVED:
                self.state = NavState.ARRIVED
                print(f"\n{'='*50}\n  Goal Reached\n  Pos: ({self.odometry.x:.3f}, {self.odometry.y:.3f})\n  Replans: {self.replan_count}\n{'='*50}\n")
            return (0, 0)

        if self.state in (NavState.IDLE, NavState.ARRIVED):
            return (0, 0)

        if self.state == NavState.PLANNING:
            if self._plan_path():
                self.state = NavState.NAVIGATING
            else:
                print("[NAV] No path found, retrying...")
            return (0, 0)

        if self.state == NavState.BLOCKED:
            self.blocked_timer += 1
            if self.blocked_timer < 50:
                return (-SLOW_SPEED, -SLOW_SPEED)
            self.blocked_timer = 0
            self.scan_timer    = 0
            self.state         = NavState.SCANNING
            print("[NAV] Backed up, scanning...")
            return (0, 0)

        if self.state == NavState.SCANNING:
            self.scan_timer += 1
            if self.scan_timer < 65:
                t = TURN_SPEED * 0.8 * self.scan_direction
                return (-t, t)
            self.scan_timer      = 0
            self.scan_direction *= -1
            self.state           = NavState.PLANNING
            return (0, 0)

        if self.state == NavState.NAVIGATING:
            if self.front_dist < OBSTACLE_STOP_DIST:
                print(f"[NAV] Obstacle at {self.front_dist:.2f}m - backing up")
                self.state         = NavState.BLOCKED
                self.blocked_timer = 0
                return (0, 0)

            if self._is_path_blocked():
                print("[NAV] Path blocked - replanning")
                self.state = NavState.PLANNING
                return (0, 0)

            self._advance_waypoint()

            if self.waypoint_idx >= len(self.path_world):
                self.state = NavState.PLANNING
                return (0, 0)

            target_x, target_y = self._current_waypoint()
            heading_error = self.angle_error(self.angle_to(target_x, target_y))
            speed = SLOW_SPEED if self.front_dist < OBSTACLE_SLOW_DIST else DRIVE_SPEED

            if abs(heading_error) > math.radians(45):
                d = 1 if heading_error > 0 else -1
                return (-TURN_SPEED * d * 0.7, TURN_SPEED * d * 0.7)

            turn_cmd   = max(-TURN_SPEED, min(TURN_SPEED, heading_error * 1.5))
            turn_factor = 1.0 - min(abs(heading_error) / math.pi, 0.5)
            fwd_speed   = speed * turn_factor

            return (fwd_speed - turn_cmd * 0.4, fwd_speed + turn_cmd * 0.4)

        return (0, 0)

    def get_status(self):
        dist = self.distance_to_goal()
        wp   = f"{self.waypoint_idx}/{len(self.path_world)}" if self.path_world else "-"
        return f"{self.state:10s} | Goal: {dist:.2f}m | WP: {wp} | Front: {self.front_dist:.2f}m"


# Webots setup
robot    = Robot()
timestep = int(robot.getBasicTimeStep())
dt       = timestep / 1000.0

print("=" * 60)
print(f"  Goal: ({GOAL_X}, {GOAL_Y}) | Arena: {ARENA_WIDTH}x{ARENA_LENGTH}m | dt: {dt}s")
print("=" * 60)

motor_FL = robot.getDevice("Front_Left_Motor")
motor_FR = robot.getDevice("Front_Right_Motor")
motor_RL = robot.getDevice("Rear_Left_Motor")
motor_RR = robot.getDevice("Rear_Right_Motor")

for m in [motor_FL, motor_FR, motor_RL, motor_RR]:
    m.setPosition(float("inf"))
    m.setVelocity(0)

try:
    ps_FL = motor_FL.getPositionSensor()
    ps_FR = motor_FR.getPositionSensor()
    ps_RL = motor_RL.getPositionSensor()
    ps_RR = motor_RR.getPositionSensor()
    for ps in [ps_FL, ps_FR, ps_RL, ps_RR]:
        ps.enable(timestep)
    encoders_ok = True
    print("[OK] Encoders")
except Exception as e:
    encoders_ok = False
    print(f"[ERROR] Encoders: {e}")

try:
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    imu_ok = True
    print("[OK] IMU")
except Exception as e:
    imu = None
    imu_ok = False
    print(f"[ERROR] IMU: {e}")

try:
    range_finder = robot.getDevice("ZED_Cam")
    range_finder.enable(timestep)
    rf_width  = range_finder.getWidth()
    rf_height = range_finder.getHeight()
    rf_ok = True
    print(f"[OK] RangeFinder ({rf_width}x{rf_height})")
except Exception as e:
    range_finder = None
    rf_width = rf_height = 0
    rf_ok = False
    print(f"[WARNING] RangeFinder: {e}")

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

grid     = OccupancyGrid(ARENA_WIDTH, ARENA_LENGTH, GRID_RESOLUTION)
odometry = DeadReckoning()
odometry.set_position(ROBOT_START_X, ROBOT_START_Y)
navigator = Navigator(grid, odometry, GOAL_X, GOAL_Y)

viz = None
if MATPLOTLIB_AVAILABLE and SHOW_VISUALIZATION:
    viz = MapVisualizer(grid, SCRIPT_DIR)
    viz.set_goal(GOAL_X, GOAL_Y)

print(f"[OK] Grid: {grid.cols}x{grid.rows} | Start: ({ROBOT_START_X}, {ROBOT_START_Y}) | Goal: ({GOAL_X}, {GOAL_Y})")
print("Controls: SPACE=Start | G=Save | W/A/S/D=Manual")
print("=" * 60)


def set_motors(left, right):
    motor_FL.setVelocity(left)
    motor_RL.setVelocity(left)
    motor_FR.setVelocity(right)
    motor_RR.setVelocity(right)

def stop_motors():
    set_motors(0, 0)


step = 0
last_save_step = 0
autonomous_started = False

while robot.step(timestep) != -1:
    step += 1

    # Odometry update
    if encoders_ok and imu_ok:
        _, _, yaw = imu.getRollPitchYaw()
        robot_x, robot_y, robot_heading, _ = odometry.update(
            ps_FL.getValue(), ps_FR.getValue(),
            ps_RL.getValue(), ps_RR.getValue(),
            yaw, dt
        )
    else:
        robot_x = robot_y = robot_heading = 0.0

    # Map update from rangefinder
    depth_data = None
    if rf_ok:
        depth_data = range_finder.getRangeImage()
        if depth_data is not None:
            obstacles = depth_image_to_obstacles(depth_data, rf_width, rf_height,
                                                  robot_x, robot_y, robot_heading)
            rob_col, rob_row = grid.world_to_cell(robot_x, robot_y)
            for wx, wy, is_obs in obstacles:
                hit_col, hit_row = grid.world_to_cell(wx, wy)
                if grid.in_bounds(hit_col, hit_row):
                    grid.cast_ray(rob_col, rob_row, hit_col, hit_row, is_obs)

            navigator.set_front_distance(get_front_obstacle_distance(depth_data, rf_width, rf_height))

    # Keyboard input
    key = keyboard.getKey()
    manual_override = False

    if key == ord(' '):
        if not autonomous_started:
            navigator.start()
            autonomous_started = True
    elif key == ord('G'):
        rob_col, rob_row = grid.world_to_cell(robot_x, robot_y)
        grid.save_csv(MAP_LIVE_PATH, rob_col, rob_row)
        print(f"\n[SAVE] Pos: ({robot_x:.2f}, {robot_y:.2f})")
        stop_motors()
        continue
    elif key == ord('W'):
        manual_override = True; set_motors(DRIVE_SPEED, DRIVE_SPEED)
    elif key == ord('S'):
        manual_override = True; set_motors(-DRIVE_SPEED, -DRIVE_SPEED)
    elif key == ord('A'):
        manual_override = True; set_motors(-TURN_SPEED, TURN_SPEED)
    elif key == ord('D'):
        manual_override = True; set_motors(TURN_SPEED, -TURN_SPEED)

    # Autonomous navigation
    if not manual_override:
        if autonomous_started:
            left, right = navigator.update(step)
            set_motors(left, right)
        else:
            stop_motors()

    # Visualization
    if viz and step % VIZ_UPDATE_INTERVAL == 0:
        viz.set_path(navigator.path_world)
        viz.set_heading(robot_heading)
        viz.set_title(f"Navigation - {navigator.get_status()}")
        viz.update(robot_x, robot_y)

    # Periodic map save
    if step - last_save_step >= MAP_SAVE_INTERVAL:
        rob_col, rob_row = grid.world_to_cell(robot_x, robot_y)
        grid.save_csv(MAP_LIVE_PATH, rob_col, rob_row)
        last_save_step = step

    if step % 25 == 0:
        status = navigator.get_status() if autonomous_started else "SPACE to start"
        print(f"Step {step:5d} | ({robot_x:+5.2f}, {robot_y:+5.2f}) {math.degrees(robot_heading):+6.1f}° | {status}", end='\r')


# Shutdown
print("\n" + "=" * 60)
rob_col, rob_row = grid.world_to_cell(robot_x, robot_y)
grid.save_csv(MAP_FINAL_PATH, rob_col, rob_row)
print(f"Final pos: ({robot_x:.3f}, {robot_y:.3f}) | Goal: ({GOAL_X:.3f}, {GOAL_Y:.3f})")
print(f"Distance: {navigator.distance_to_goal():.3f}m | Replans: {navigator.replan_count}")
print("=" * 60)

if viz:
    viz.close()