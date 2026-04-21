import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
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
                ('inflation_radius', 1),
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
        self.inflation_radius = self.get_parameter('inflation_radius').value
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
            self.mark_occupied(wx, wy)
            gx, gy = self.world_to_grid(wx, wy)
            if self._in_bounds(gx, gy):
                self.static_grid[gy][gx] = 1

        # ===== Publishers =====
        self._event_pub = self.create_publisher(FSMEvent, f'/{self.uav_id}/fsm/event', 10)

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

        self.get_logger().debug(f'WorldModelNode ready for {self.uav_id}')

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
        gx, gy = self.world_to_grid(wx, wy)
        r = self.inflation_radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if self._in_bounds(nx, ny):
                    self.grid[ny][nx] = 1

    def mark_free(self, wx, wy):
        gx, gy = self.world_to_grid(wx, wy)
        if self._in_bounds(gx, gy) and self.static_grid[gy][gx] == 0:
            self.grid[gy][gx] = 0

    def _clear_dynamic_obstacle(self, wx, wy):
        gx, gy = self.world_to_grid(wx, wy)
        r = self.inflation_radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = gx + dx, gy + dy
                if self._in_bounds(nx, ny) and self.static_grid[ny][nx] == 0:
                    self.grid[ny][nx] = -1

    # ===== Sensor-Based Updates =====

    def _on_laser_scan(self, msg):
        if self.own_pose is None:
            return

        sx, sy, yaw = self.own_pose

        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue

            # beam angle in world frame
            angle = yaw + msg.angle_min + i * msg.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)

            # march along ray marking free space
            d = 0.0
            while d < r - self.resolution:
                wx = sx + d * dx
                wy = sy + d * dy
                gx, gy = self.world_to_grid(wx, wy)
                if self._in_bounds(gx, gy) and self.static_grid[gy][gx] == 0:
                    self.grid[gy][gx] = 0
                d += self.resolution

            # mark hit point as occupied
            self.mark_occupied(sx + r * dx, sy + r * dy)

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

        if uav_id in self.dynamic_obstacles:
            old_x, old_y = self.dynamic_obstacles[uav_id]
            self._clear_dynamic_obstacle(old_x, old_y)

        self.mark_occupied(x, y)
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
