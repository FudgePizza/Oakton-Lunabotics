import math
import os
import odometry

MATPLOTLIB_AVAILABLE = False
plt = None

for _backend in ['Qt5Agg', 'TkAgg', 'wxAgg', 'Agg']:
    try:
        import matplotlib
        matplotlib.use(_backend)
        import matplotlib.pyplot as plt
        _test_fig = plt.figure()
        plt.close(_test_fig)
        MATPLOTLIB_AVAILABLE = True
        print(f"[OK] matplotlib backend: {_backend}")
        break
    except Exception:
        pass

if not MATPLOTLIB_AVAILABLE:
    print("[WARNING] matplotlib not available")

VIZ_UPDATE_INTERVAL = 10
SHOW_VISUALIZATION  = True

_goal_x = 5.0
_goal_y = 3.0

viz = None
_step = 0
_script_dir = os.path.dirname(os.path.abspath(__file__))

_lidar_mod = None
_path_mod  = None


def _lazy_imports():
    global _lidar_mod, _path_mod
    if _lidar_mod is None:
        try:
            import lidar_processing
            _lidar_mod = lidar_processing
        except ImportError:
            pass
    if _path_mod is None:
        try:
            import pathfinding
            _path_mod = pathfinding
        except ImportError:
            pass


class PointCloudVisualizer:
    def __init__(self, script_dir=".", goal_x=5.0, goal_y=3.0):
        self.fig = self.ax = None
        self.trajectory_x = []
        self.trajectory_y = []
        self._agg_only = False
        self._snapshot_path = os.path.join(script_dir, "map_snapshot.png")
        self._goal_x = goal_x
        self._goal_y = goal_y

        self._traj_line = None
        self._scan_scatter = None
        self._obs_scatter = None
        self._landmark_scatter = None
        self._robot_dot = None
        self._robot_arrow = None
        self._goal_dot = None
        self._steer_line = None

        if MATPLOTLIB_AVAILABLE:
            try:
                self._init_figure()
            except Exception as e:
                print(f"[WARNING] Viz init failed: {e}")

    def _init_figure(self):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        try:
            self.fig.canvas.manager.set_window_title('Lunabotics - Point Cloud')
        except AttributeError:
            pass

        self.ax.set_xlim(-1, 11)
        self.ax.set_ylim(-1, 11)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_xticks(range(-1, 12))
        self.ax.set_yticks(range(-1, 12))
        self.ax.grid(True, alpha=0.3)

        self._goal_dot, = self.ax.plot([self._goal_x], [self._goal_y], 'g*',
                                        markersize=15, zorder=10, label='Goal')
        self._traj_line, = self.ax.plot([], [], 'b-', linewidth=1,
                                         alpha=0.4, zorder=3, label='Trajectory')
        self._scan_scatter = self.ax.scatter([], [], s=2, c='lightgray',
                                              zorder=4, label='Ground')
        self._obs_scatter = self.ax.scatter([], [], s=8, c='black',
                                             zorder=5, label='Obstacle')
        self._landmark_scatter = self.ax.scatter([], [], s=60, c='red',
                                                   marker='o', edgecolors='darkred',
                                                   linewidths=1.5, zorder=8,
                                                   label='Rock landmark')
        self._robot_dot, = self.ax.plot([], [], 'bo', markersize=10, zorder=9)
        self._robot_arrow = self.ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                              arrowprops=dict(arrowstyle='->',
                                                              color='blue', lw=2),
                                              zorder=11)
        self._steer_line, = self.ax.plot([], [], 'g-', linewidth=2,
                                          alpha=0.8, zorder=7, label='Steer')

        self.ax.legend(loc='upper right', fontsize=8)
        plt.tight_layout()
        self.fig.canvas.draw_idle()
        try:
            self.fig.canvas.flush_events()
        except Exception:
            pass

        backend_name = matplotlib.get_backend().lower()
        self._agg_only = ('agg' in backend_name and 'qt' not in backend_name
                          and 'tk' not in backend_name and 'wx' not in backend_name)

    def update(self, rx, ry, rh, title=""):
        if not MATPLOTLIB_AVAILABLE or self.ax is None:
            return
        _lazy_imports()
        try:
            self.trajectory_x.append(rx)
            self.trajectory_y.append(ry)
            self._traj_line.set_data(self.trajectory_x, self.trajectory_y)

            self._robot_dot.set_data([rx], [ry])
            arrow_len = 0.3
            self._robot_arrow.set_position((rx, ry))
            self._robot_arrow.xy = (rx + arrow_len * math.cos(rh),
                                    ry + arrow_len * math.sin(rh))

            if _lidar_mod:
                scan = _lidar_mod.get_scan_world()
                if scan:
                    ground_x = [p[0] for p in scan if p[2]]
                    ground_y = [p[1] for p in scan if p[2]]
                    obs_x = [p[0] for p in scan if not p[2]]
                    obs_y = [p[1] for p in scan if not p[2]]

                    if ground_x:
                        self._scan_scatter.set_offsets(list(zip(ground_x, ground_y)))
                    else:
                        self._scan_scatter.set_offsets([(float('nan'), float('nan'))])

                    if obs_x:
                        self._obs_scatter.set_offsets(list(zip(obs_x, obs_y)))
                    else:
                        self._obs_scatter.set_offsets([(float('nan'), float('nan'))])
                else:
                    self._scan_scatter.set_offsets([(float('nan'), float('nan'))])
                    self._obs_scatter.set_offsets([(float('nan'), float('nan'))])

                lms = _lidar_mod.get_landmarks()
                if lms:
                    self._landmark_scatter.set_offsets(
                        [(lm['x'], lm['y']) for lm in lms])
                else:
                    self._landmark_scatter.set_offsets([(float('nan'), float('nan'))])

            if _path_mod:
                path = _path_mod.get_path()
                if path and len(path) == 2:
                    self._steer_line.set_data([path[0][0], path[1][0]],
                                               [path[0][1], path[1][1]])

            if title:
                self.ax.set_title(title)

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

            if self._agg_only:
                self.fig.savefig(self._snapshot_path, dpi=90, bbox_inches='tight')

        except Exception as e:
            print(f"[VIZ] Error: {e}")

    def close(self):
        if MATPLOTLIB_AVAILABLE and self.fig:
            try:
                if not self._agg_only:
                    plt.ioff()
                self.fig.savefig(self._snapshot_path, dpi=120, bbox_inches='tight')
                print(f"[VIZ] Saved {self._snapshot_path}")
                plt.close(self.fig)
            except Exception as e:
                print(f"[VIZ] Close error: {e}")


def setup(script_dir=None, goal_x=None, goal_y=None):
    global viz, _step, _script_dir, _goal_x, _goal_y

    if goal_x is not None:
        _goal_x = goal_x
    if goal_y is not None:
        _goal_y = goal_y
    if script_dir is not None:
        _script_dir = script_dir

    if SHOW_VISUALIZATION and MATPLOTLIB_AVAILABLE:
        viz = PointCloudVisualizer(_script_dir, _goal_x, _goal_y)
    _step = 0


def update():
    global _step
    _step += 1
    if viz and _step % VIZ_UPDATE_INTERVAL == 0:
        rx, ry, rh = odometry.get_pose()
        _lazy_imports()
        title = ""
        if _path_mod:
            title = _path_mod.get_status()
        viz.update(rx, ry, rh, title)


def get_grid():
    return None


def save_final():
    pass