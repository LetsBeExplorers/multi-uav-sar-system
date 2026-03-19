import heapq
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path, Odometry


GridCell = Tuple[int, int]


class AStarNavigationNode(Node):
    def __init__(self):
        super().__init__('astar_navigation_node')

        # Parameters for easier integration later
        self.declare_parameter('waypoint_topic', '/input_waypoints')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('uav_name', 'x1')
        
        uav = self.get_parameter('uav_name').value
        waypoint_topic = f'/{uav}/nav/waypoints'
        path_topic = f'/{uav}/nav/planned_path'

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.waypoint_sub = self.create_subscription(
            PoseArray,
            waypoint_topic,
            self.waypoint_callback,
            10
        )

        self.create_subscription(
            Odometry,
            f'/{uav}/state/odom',
            self.odom_callback,
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
            (8,4), (8,6),
        ]

        for x, y in self.obstacles:
            gx, gy = self.world_to_grid(x, y)
            self.grid[gy][gx] = 1

        # Current UAV position
        self.current_position = self.world_to_grid(-10, -10)

        # Waypoints received from topic
        self.waypoints: List[GridCell] = []
        self.current_waypoint_index = 0
        self.received_waypoints = False

        # Track planning cycles
        self.plan_cycle = 0

        self.timer = self.create_timer(2.0, self.plan_and_advance)

        self.get_logger().info('A* obstacle navigation node started')
        self.get_logger().info(f'Listening for waypoints on: {waypoint_topic}')
        self.get_logger().info(f'Publishing planned path on: {path_topic}')

    def odom_callback(self, msg):
        x = int(round(msg.pose.pose.position.x))
        y = int(round(msg.pose.pose.position.y))
        self.current_position = self.world_to_grid(x, y)

    def world_to_grid(self, x, y):
        return (x + 10, y + 10)

    def waypoint_callback(self, msg: PoseArray):

        if self.received_waypoints:
            return   # ignore duplicates

        self.waypoints = []

        for pose in msg.poses:
            wx = int(round(pose.position.x))
            wy = int(round(pose.position.y))

            gx, gy = self.world_to_grid(wx, wy)
            self.waypoints.append((gx, gy))

        self.current_waypoint_index = 0
        self.received_waypoints = True

        self.get_logger().info(f'Received waypoints: {self.waypoints}')

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
        dx = abs(self.current_position[0] - goal[0])
        dy = abs(self.current_position[1] - goal[1])
        return dx <= 1 and dy <= 1   # small tolerance

    def plan_and_advance(self):
        if not self.received_waypoints:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed')
            return

        goal = self.waypoints[self.current_waypoint_index]
        self.plan_cycle += 1

#        if self.plan_cycle == 2:
#            self.add_dynamic_obstacle()

        path = self.astar(self.current_position, goal)

        if path is None:
            self.get_logger().warn(f'No valid path found to waypoint {goal}')
            return

        self.get_logger().info(
            f'Planning from {self.current_position} to waypoint {goal}: {path}'
        )

        path_msg = self.build_path_msg(path)
        self.path_pub.publish(path_msg)

        if self.reached_goal(goal):
            self.current_waypoint_index += 1

        if self.current_waypoint_index < len(self.waypoints):
            self.get_logger().info(
                f'Moving on to next waypoint: {self.waypoints[self.current_waypoint_index]}'
            )
        else:
            self.get_logger().info('Mission complete')


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
