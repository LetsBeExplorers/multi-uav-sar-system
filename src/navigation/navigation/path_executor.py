import csv
import math
import os
from datetime import datetime, timezone

from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent
from std_msgs.msg import Empty, String


class PathExecutorNode(Node):

    def __init__(self):
        super().__init__('path_executor')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('speed', 2.0),
                ('waypoint_threshold', 0.2),
                ('lookahead', 5),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.speed = self.get_parameter('speed').value
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.lookahead = self.get_parameter('lookahead').value

        # ===== State =====
        self.current_path = []       # list of (x, y) tuples from A* planned path
        self.current_index = 0
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.uav_z = 0.0
        self.home_x = None
        self.home_y = None
        self.is_returning = False
        self.path_cells_total = 0
        self.path_cells_traversed = 0
        self._target_z = 1.0   # cruise altitude matching waypoint z

        qos_transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ===== Publishers =====
        self._cmd_pub = self.create_publisher(
            Twist, f'/{self.uav_id}/platform/cmd_vel', 10)
        self._reached_pub = self.create_publisher(
            Empty, f'/{self.uav_id}/nav/reached_coverage_waypoint', 10)
        self._event_pub = self.create_publisher(
            FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        # go_home() publishes a single-pose PoseArray back into A* for return planning
        self._waypoint_pub = self.create_publisher(
            PoseArray, f'/{self.uav_id}/nav/waypoints', qos_transient)
        self._status_pub = self.create_publisher(String, '/mission/status', 10)

        # ===== Subscribers =====
        self.create_subscription(
            Path,
            f'/{self.uav_id}/nav/planned_path',
            self._on_path_received,
            qos_transient
        )
        self.create_subscription(
            Odometry,
            f'/{self.uav_id}/state/odom',
            self._on_pose_update,
            10
        )
        self.create_subscription(
            FSMEvent,
            f'/{self.uav_id}/fsm/command',
            self._on_fsm_command,
            10
        )
        self.create_subscription(
            Empty,
            '/mission/stop',
            self._on_stop,
            10
        )

        # ===== Timer =====
        self.create_timer(0.1, self._move_step)

    # ===== Pose Tracking =====

    def _on_pose_update(self, msg):
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y
        self.uav_z = msg.pose.pose.position.z

        if self.home_x is None:
            self.home_x = self.uav_x
            self.home_y = self.uav_y

    # ===== Path Reception =====

    def _on_path_received(self, msg):
        if not msg.poses:
            return
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # extend the path so we end at actual home, not the grid boundary
        if self.is_returning and self.home_x is not None:
            if self.current_path[-1] != (self.home_x, self.home_y):
                self.current_path.append((self.home_x, self.home_y))
        self.current_index = min(
            range(len(self.current_path)),
            key=lambda i: math.hypot(
                self.current_path[i][0] - self.uav_x,
                self.current_path[i][1] - self.uav_y
            )
        )
        self.path_cells_total += len(self.current_path)

    # ===== Execution =====

    def _move_step(self):
        if not self.current_path:
            self._stop()
            return

        # Advance index to the nearest cell ahead (forward only)
        while self.current_index + 1 < len(self.current_path):
            cx, cy = self.current_path[self.current_index]
            nx, ny = self.current_path[self.current_index + 1]
            if math.hypot(nx - self.uav_x, ny - self.uav_y) <= math.hypot(cx - self.uav_x, cy - self.uav_y):
                self.current_index += 1
                self.path_cells_traversed += 1
            else:
                break

        # Signal completion when within threshold of the final cell
        if self.current_index == len(self.current_path) - 1:
            cx, cy = self.current_path[-1]
            if math.hypot(cx - self.uav_x, cy - self.uav_y) < self.waypoint_threshold:
                self._stop()
                # signal A* that a coverage waypoint segment is complete
                self._reached_pub.publish(Empty())

                if self.is_returning:
                    self.is_returning = False
                    event = FSMEvent()
                    event.uav_id = self.uav_id
                    event.event = 'HOME_REACHED'
                    event.timestamp = self.get_clock().now().nanoseconds / 1e9
                    self._event_pub.publish(event)
                    self._log_metrics()

                self.current_path = []
                self.current_index = 0
                return

        # steer toward lookahead point for smooth motion
        lookahead = self.lookahead
        for k in range(1, min(self.lookahead, len(self.current_path) - self.current_index - 1)):
            ax, ay = self.current_path[self.current_index + k - 1]
            bx, by = self.current_path[self.current_index + k]
            cx, cy = self.current_path[self.current_index + k + 1]
            if (bx - ax, by - ay) != (cx - bx, cy - by):
                lookahead = k
                break

        target_index = min(self.current_index + lookahead, len(self.current_path) - 1)
        tx, ty = self.current_path[target_index]
        dx = tx - self.uav_x
        dy = ty - self.uav_y
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist > 0:
            cmd.linear.x = dx / dist * self.speed
            cmd.linear.y = dy / dist * self.speed
        cmd.linear.z = max(-1.0, min(1.0, (self._target_z - self.uav_z) * 2.0))
        self._cmd_pub.publish(cmd)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    # ===== Home Return =====

    def go_home(self):
        """Send home position as a single-waypoint PoseArray so A* plans the return path."""
        if self.home_x is None:
            return
        self.is_returning = True
        self.current_path = []
        self.current_index = 0

        msg = PoseArray()
        pose = Pose()
        pose.position.x = float(self.home_x)
        pose.position.y = float(self.home_y)
        pose.orientation.w = 1.0
        msg.poses.append(pose)
        self._waypoint_pub.publish(msg)

        self.get_logger().info(
            f'[{self.uav_id}] returning home ({self.home_x:.1f}, {self.home_y:.1f})')

    def _on_fsm_command(self, msg):
        if msg.event == 'GO_HOME':
            self.go_home()

    # ===== Stop Handler =====

    def _on_stop(self, msg):
        self.current_path = []
        self.current_index = 0
        self.is_returning = False
        self._stop()

    # ===== Metrics =====

    def _log_metrics(self):
        ratio = (self.path_cells_traversed / self.path_cells_total
                 if self.path_cells_total > 0 else 0.0)
        log_dir = os.path.expanduser('~/.ros/path_metrics')
        os.makedirs(log_dir, exist_ok=True)
        csv_path = os.path.join(log_dir, f'{self.uav_id}_path_metrics.csv')
        write_header = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow([
                    'uav_id', 'timestamp', 'path_cells_total',
                    'path_cells_traversed', 'completion_ratio',
                ])
            writer.writerow([
                self.uav_id,
                datetime.now(timezone.utc).isoformat(),
                self.path_cells_total,
                self.path_cells_traversed,
                f'{ratio:.4f}',
            ])


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
