import heapq
from typing import Dict, List, Optional, Tuple

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, Int32, String


GridCell = Tuple[int, int]


class AStarNavigationNode(Node):
    def __init__(self):
        super().__init__('astar_navigation_node')

        # Parameters for easier integration later
        self.declare_parameter('waypoint_topic', '/input_waypoints')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('uav_name', 'x1')
        
        self.uav = self.get_parameter('uav_name').value
        waypoint_topic = f'/{self.uav}/nav/waypoints'
        path_topic = f'/{self.uav}/nav/planned_path'

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, path_topic, qos)

        self.waypoint_sub = self.create_subscription(
            PoseArray,
            waypoint_topic,
            self.waypoint_callback,
            qos
        )

        self.create_subscription(
            Odometry,
            f'/{self.uav}/state/odom',
            self.odom_callback,
            10
        )

        # Publishes Mission Status and # of Waypoints
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.waypoint_count_pub = self.create_publisher(
            Int32,
            f'/{self.uav}/nav/waypoint_count',
            10
        )

        # Grid dimensions
        self.width = 21
        self.height = 21
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # Initial static obstacles
        self.obstacles = [
            (2,2), (2,4),
            (5,3), (5,5),
            (3,1), (4,2), (3,3)
        ]

        for x, y in self.obstacles:
            gx, gy = self.world_to_grid(x, y)
            self.grid[gy][gx] = 1

        # Waypoints received from topic
        self.current_position = None
        self.waypoints = []
        self.started = False

    # Just grabs position of drone
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x + 10, y + 10)

    def world_to_grid(self, x, y):
        return (x + 10, y + 10)

    def waypoint_callback(self, msg: PoseArray):
        if self.current_position is None:
            return

        waypoints = [
            self.world_to_grid(
                int(round(p.position.x)),
                int(round(p.position.y))
            )
            for p in msg.poses
        ]

        start = (int(self.current_position[0]), int(self.current_position[1]))
        full_path = []

        for goal in waypoints:
            partial = self.astar(start, goal)
            if partial:
                full_path.extend(partial)
                start = goal

        if full_path:
            path_msg = self.build_path_msg(full_path)
            self.path_pub.publish(path_msg)

            # Publish waypoint count to nodes
            count_msg = Int32()
            count_msg.data = len(full_path)
            self.waypoint_count_pub.publish(count_msg)

            # Publish waypoints to console
            msg = String()
            msg.data = f"[{self.uav}] WAYPOINTS: {len(full_path)}"
            self.status_pub.publish(msg)

    def heuristic(self, a: GridCell, b: GridCell) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, cell: GridCell) -> List[GridCell]:
        x, y = cell
        candidates = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]

        valid = []
        for nx, ny in candidates:
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if self.grid[ny][nx] == 0:
                    valid.append((nx, ny))
        return valid

    def reconstruct_path(
        self,
        came_from: Dict[GridCell, GridCell],
        current: GridCell
    ) -> List[GridCell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def astar(self, start: GridCell, goal: GridCell) -> Optional[List[GridCell]]:
        open_heap: List[Tuple[int, GridCell]] = []
        heapq.heappush(open_heap, (0, start))

        came_from: Dict[GridCell, GridCell] = {}
        g_score: Dict[GridCell, int] = {start: 0}
        f_score: Dict[GridCell, int] = {start: self.heuristic(start, goal)}

        open_set = {start}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            open_set.discard(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        heapq.heappush(open_heap, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)

        return None

    def build_path_msg(self, cells: List[GridCell]) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in cells:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x - 10)
            pose.pose.position.y = float(y - 10)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg

    def add_dynamic_obstacle(self):
        dynamic_obstacles = [(7, 3), (7, 4), (7, 5)]

        for x, y in dynamic_obstacles:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y][x] = 1

        self.get_logger().info(
            f'Dynamic obstacle added at cells: {dynamic_obstacles}'
        )

    def reached_goal(self, goal):
        # Convert grid back to world coords
        goal_x = goal[0] - 10
        goal_y = goal[1] - 10

        current_x = self.current_position[0] - 10
        current_y = self.current_position[1] - 10

        dx = goal_x - current_x
        dy = goal_y - current_y

        dist = (dx**2 + dy**2) ** 0.5

        return dist < 0.2 # tolerance

    def compute_and_publish_full_path(self):
        if not self.waypoints or self.current_position is None:
            return
        full_path = []
        start = (int(self.current_position[0]), int(self.current_position[1]))
        for idx in range(self.current_waypoint_index, len(self.waypoints)):
            goal = self.waypoints[idx]
            partial_path = self.astar(start, goal)
            if partial_path:
                full_path.extend(partial_path)
                start = goal
        if full_path:
            path_msg = self.build_path_msg(full_path)
            self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
