import math

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sar_msgs.msg import FSMEvent
from sar_msgs.srv import GetOccupancyGrid
from sensor_msgs.msg import LaserScan


class WorldModelNode(Node):

    def __init__(self):
        super().__init__('world_model')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('uav_id', 'x1'),
                ('num_uavs', 3),
                ('grid_width', 21),
                ('grid_height', 21),
                ('resolution', 1.0),
                ('origin_x', -10.0),
                ('origin_y', -10.0),
                ('static_obstacles', []),
                ('collision_threshold', 0.5),
            ]
        )

        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.collision_threshold = self.get_parameter('collision_threshold').value

        # ===== State =====
        # -1 = unknown, 0 = free, 1 = occupied
        self.grid = [[-1] * self.grid_width for _ in range(self.grid_height)]
        self.static_grid = [[0] * self.grid_width for _ in range(self.grid_height)]
        self.dynamic_obstacles = {}
        self.own_pose = None
        self.collision_count = 0

        # ===== Static Obstacles =====
        # param is a flat list [x1, y1, x2, y2, ...]
        obs_flat = self.get_parameter('static_obstacles').value
        for i in range(0, len(obs_flat) - 1, 2):
            wx, wy = float(obs_flat[i]), float(obs_flat[i + 1])
            gx, gy = self.world_to_grid(wx, wy)
            if self._in_bounds(gx, gy):
                self.static_grid[gy][gx] = 1
                self.grid[gy][gx] = 4
        
        qos_transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ===== Publishers =====
        self._event_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)
        self._grid_pub = self.create_publisher(OccupancyGrid, f'/{self.uav_id}/world_model/grid', qos_transient)

        # ===== Subscribers =====
        self.create_subscription(
            Odometry,
            f'/{self.uav_id}/state/odom',
            self._on_own_pose_update,
            10
        )
        self.create_subscription(
            LaserScan,
            f'/{self.uav_id}/scan',
            self._on_laser_scan,
            10
        )

        all_uavs = [f'x{i + 1}' for i in range(self.num_uavs)]
        for other_id in all_uavs:
            if other_id != self.uav_id:
                self.create_subscription(
                    Odometry,
                    f'/{other_id}/state/odom',
                    lambda msg, uid=other_id: self._on_dynamic_obstacle_update(uid, msg),
                    10
                )

        # ===== Service =====
        self.create_service(
            GetOccupancyGrid,
            f'/{self.uav_id}/world_model/get_grid',
            self._get_occupancy_grid
        )

        # ===== Grid Publisher Timer =====
        self.create_timer(0.2, self._publish_grid)

    # ===== Grid Management =====

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return wx, wy

    def _in_bounds(self, gx, gy):
        return 0 <= gx < self.grid_width and 0 <= gy < self.grid_height

    def mark_occupied(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        if self._in_bounds(gx, gy):
            if self.static_grid[gy][gx] == 0:
                if self.grid[gy][gx] != -2:  # don't mark occupied if already confirmed free
                    self.grid[gy][gx] = 4

    def mark_free(self, wx, wy):
        gx, gy = self.world_to_grid(wx, wy)
        if self._in_bounds(gx, gy) and self.static_grid[gy][gx] == 0:
            self.grid[gy][gx] = 0

    # ===== Cell Marking =====

    def _is_freeable(self, gx, gy):
        return (
            self._in_bounds(gx, gy) and
            self.static_grid[gy][gx] == 0 and
            self.grid[gy][gx] != 4
        )

    def _apply_free(self, gx, gy):
        if self.grid[gy][gx] > -2:
            self.grid[gy][gx] -= 1

    def _apply_occupied(self, gx, gy):
        if self._in_bounds(gx, gy):
            self.grid[gy][gx] = min(self.grid[gy][gx] + 2, 4)

    # ===== Sensor-Based Updates =====

    def _bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        cells.append((x1, y1))
        return cells

    def _on_laser_scan(self, msg):
        if self.own_pose is None:
            return

        sx, sy, yaw = self.own_pose

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min:
                continue

            angle = yaw + msg.angle_min + i * msg.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)

            # determine how far to trace
            if r >= msg.range_max:
                max_range = msg.range_max
            else:
                max_range = r

            # start cell
            sx_g, sy_g = self.world_to_grid(sx, sy)

            # end point
            epsilon = 0.001
            ex = sx + (max_range - epsilon) * dx
            ey = sy + (max_range - epsilon) * dy

            ex_g, ey_g = self.world_to_grid(ex, ey)

            # trace grid cells
            cells = self._bresenham(sx_g, sy_g, ex_g, ey_g)

            # mark free space (all except last)
            for gx, gy in cells[:-1]:
                if self._is_freeable(gx, gy):
                    self._apply_free(gx, gy)

            # mark obstacle (last cell only if hit)
            if r < msg.range_max:
                gx, gy = self.world_to_grid(
                    sx + r * dx,
                    sy + r * dy
                )
                self._apply_occupied(gx, gy)

    # ===== Dynamic Obstacles =====

    def _on_own_pose_update(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        self.own_pose = (x, y, yaw)

    def _on_dynamic_obstacle_update(self, uav_id, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        footprint = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]

        # clear the full footprint at the old position to avoid ghost cells
        if uav_id in self.dynamic_obstacles:
            old_x, old_y = self.dynamic_obstacles[uav_id]
            ogx, ogy = self.world_to_grid(old_x, old_y)
            for dx, dy in footprint:
                nx, ny = ogx + dx, ogy + dy
                if self._in_bounds(nx, ny) and self.static_grid[ny][nx] == 0:
                    self.grid[ny][nx] = 0

        gx, gy = self.world_to_grid(x, y)

        for dx, dy in footprint:
            nx, ny = gx + dx, gy + dy
            if self._in_bounds(nx, ny):
                self.grid[ny][nx] = 4

        self.dynamic_obstacles[uav_id] = (x, y)

        if self.own_pose is not None:
            ox, oy, _ = self.own_pose
            dist = math.hypot(x - ox, y - oy)
            if dist < self.collision_threshold:
                self.collision_count += 1
                event = FSMEvent()
                event.uav_id = self.uav_id
                event.event = 'COLLISION_RISK'
                event.timestamp = self.get_clock().now().nanoseconds / 1e9
                self._event_pub.publish(event)

    # ===== Grid Publisher =====

    def _publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        msg.info.resolution = float(self.resolution)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.orientation.w = 1.0
        flat = []
        for row in self.grid:
            for val in row:
                if val >= 2:
                    flat.append(100)   # occupied
                elif val <= -2:
                    flat.append(0)     # free
                else:
                    flat.append(-1)    # unknown

        msg.data = flat
        self._grid_pub.publish(msg)

    # ===== Service Handler =====

    def _get_occupancy_grid(self, request, response):
        flat = []
        for row in self.grid:
            flat.extend(row)
        response.grid = flat
        response.width = self.grid_width
        response.height = self.grid_height
        response.resolution = float(self.resolution)
        response.origin_x = float(self.origin_x)
        response.origin_y = float(self.origin_y)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = WorldModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
