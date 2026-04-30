import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from sar_msgs.msg import DetectionEvent
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        # ===== Parameters =====
        self.declare_parameter('uav_ids', ['x1', 'x2', 'x3'])
        self.declare_parameter('grid_source', 'x2')  # which UAV's world_model to display
        self.uav_ids = self.get_parameter('uav_ids').value
        grid_source = self.get_parameter('grid_source').value

        # ===== State =====
        self.paths = {uid: [] for uid in self.uav_ids}
        self.positions = {uid: None for uid in self.uav_ids}
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.resolution = 1.0
        self.current_path = None
        self.targets = []  # store confirmed targets
        self.goal = None
        self.grids = {}
        self.colors = {
            'x1': 'red',
            'x2': 'green',
            'x3': 'blue',
        }
        self.ground_truth_targets = []

        # ===== Subscribers =====

        for uid in self.uav_ids:
            self.create_subscription(
                Odometry,
                f'/{uid}/state/odom',
                lambda msg, u=uid: self.odom_callback(u, msg),
                10
            )

            self.create_subscription(
                OccupancyGrid,
                f'/{uid}/world_model/grid',
                lambda msg, u=uid: self.grid_callback(u, msg),
                10
            )

        self.create_subscription(
            Path,
            f'/{grid_source}/nav/planned_path',
            self.path_callback,
            10
        )

        self.create_subscription(
            DetectionEvent,
            '/targets/confirmed',
            self._target_callback,
            10
        )

        self.create_subscription(
            DetectionEvent,
            '/mission/targets',
            self._mission_target_callback,
            10
        )

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

    # ===== Callbacks =====

    def _mission_target_callback(self, msg):
        self.ground_truth_targets.append((msg.x, msg.y))

    def grid_callback(self, uid, msg):
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.grids[uid] = (data, msg.info)

        # use first grid as reference frame
        if len(self.grids) == 1:
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.resolution = msg.info.resolution
            self.width = msg.info.width
            self.height = msg.info.height

        self._draw()

    def path_callback(self, msg):
        if msg.poses:
            self.current_path = [
                (p.pose.position.x, p.pose.position.y)
                for p in msg.poses
            ]

            # also set goal = last point
            last = msg.poses[-1].pose.position
            self.goal = (last.x, last.y)
        else:
            self.current_path = None
            self.goal = None

    def odom_callback(self, uid, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[uid] = (x, y)
        path = self.paths[uid]
        # skip near-duplicate samples to keep the path light
        if not path or (x - path[-1][0]) ** 2 + (y - path[-1][1]) ** 2 > 0.04:
            path.append((x, y))

    def _target_callback(self, msg):
        # store multiple
        self.targets.append((msg.x, msg.y))

    # ===== Drawing =====

    def _merge_grids(self):
        if not self.grids:
            return None

        # assume same size/resolution for now
        grids = [g[0] for g in self.grids.values()]
        merged = np.full_like(grids[0], -1)

        for g in grids:
            known = g != -1
            merged = np.maximum.reduce(grids)

        return merged

    def _draw(self):
        if not self.grids:
            return
        data = self._merge_grids()

        self.ax.clear()

        display = np.zeros_like(data, dtype=float)
        display[data == -1] = 0.5   # unknown = gray
        display[data == 0] = 0.0    # free = black
        display[data == 100] = 1.0  # occupied = white

        extent = [
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        ]
        self.ax.imshow(display, cmap='gray', origin='lower', vmin=0, vmax=1, extent=extent)

        for uid in self.uav_ids:
            color = self.colors.get(uid, 'black')
            path = self.paths[uid]
            if path:
                xs, ys = zip(*path)
                self.ax.plot(xs, ys, '-', color=color, linewidth=1.5, label=uid)
            pos = self.positions[uid]
            if pos is not None:
                self.ax.plot(pos[0], pos[1], 'o', color=color, markersize=8,
                             markeredgecolor='black', markeredgewidth=0.5)

        if self.current_path:
            xs, ys = zip(*self.current_path)
            self.ax.plot(xs, ys, '--', color='yellow', linewidth=2, label='planned')

        if self.goal is not None:
            self.ax.plot(
                self.goal[0],
                self.goal[1],
                'X',
                color='yellow',
                markersize=12,
                markeredgecolor='black'
            )

        # draw ground truth targets (light / hollow)
        for (x, y) in self.ground_truth_targets:
            self.ax.plot(
                x, y,
                'o',
                color='cyan',
                markersize=10,
                markerfacecolor='none',
                alpha=0.4,
                linewidth=1.5
            )

        # draw confirmed targets
        label_added = False
        for (x, y) in self.targets:
            self.ax.plot(
                x,
                y,
                '*',
                color='cyan',
                markersize=15,
                markeredgecolor='black',
                label='target' if not label_added else ""
            )
            label_added = True

        self.ax.set_title('Occupancy Grid + UAV Paths')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5))
        self.ax.set_aspect('equal')
        plt.draw()
        plt.pause(0.001)

    # ===== Final Snapshot =====

    def save_final(self):
        if self.last_grid is None:
            return
        self._draw()
        out_dir = os.path.expanduser('~/.ros/map_snapshots')
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(out_dir, f'final_map_{ts}.png')
        self.fig.savefig(path, dpi=150, bbox_inches='tight')

def main():
    rclpy.init()
    node = MapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_final()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
