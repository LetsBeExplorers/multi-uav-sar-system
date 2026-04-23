import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sar_msgs.msg import MissionCoverage, UAVState
from std_msgs.msg import Empty, String


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_uavs', 3),
                ('threshold', 0.95),
            ]
        )

        num_uavs = self.get_parameter('num_uavs').value
        self.threshold = self.get_parameter('threshold').value

        # ===== State =====
        self.uav_ids = [f'x{i + 1}' for i in range(num_uavs)]
        self.mission_state = 'IDLE'
        self.uav_states = {uid: 'IDLE' for uid in self.uav_ids}
        self.uav_coverage = {uid: 0.0 for uid in self.uav_ids}

        # ===== Publishers =====
        self._start_pub = self.create_publisher(Empty, '/mission/start', 10)
        self._stop_pub = self.create_publisher(Empty, '/mission/stop', 10)
        self._coverage_pub = self.create_publisher(MissionCoverage, '/mission/coverage', 10)

        # ===== Subscribers =====
        self.create_subscription(UAVState, '/uav/state', self._on_uav_state_change, 10)
        self.create_subscription(String, '/mission/status', self._on_status, 10)

    # ===== State Tracking =====

    def _on_uav_state_change(self, msg):
        if msg.uav_id in self.uav_states:
            self.uav_states[msg.uav_id] = msg.state

        if all(s == 'IDLE' for s in self.uav_states.values()) and self.mission_state == 'RUNNING':
            self.mission_state = 'COMPLETE'

        self._refresh_dashboard()

    def _on_status(self, msg):
        text = msg.data

        if 'PROGRESS' in text:
            try:
                # Format: "[x1] PROGRESS: 50/100"
                uav_id = text.split(']')[0].lstrip('[')
                prog = text.split('PROGRESS:')[1].strip()
                visited, total = prog.split('/')
                if int(total) > 0:
                    self.uav_coverage[uav_id] = int(visited) / int(total)
                    self._publish_coverage()
            except (IndexError, ValueError):
                pass

        self._refresh_dashboard()

    # ===== Coverage Aggregation =====

    def _publish_coverage(self):
        msg = MissionCoverage()
        msg.uav_ids = self.uav_ids
        msg.coverage_ratios = [self.uav_coverage.get(uid, 0.0) for uid in self.uav_ids]
        msg.all_complete = all(r >= self.threshold for r in msg.coverage_ratios)
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._coverage_pub.publish(msg)

    # ===== Dashboard =====

    def _refresh_dashboard(self):
        print('\033[H\033[J', end='')
        print('=== MISSION STATUS ===')
        print(f'Mission: {self.mission_state}')

        if self.mission_state == 'IDLE':
            print('\nWaiting for start command...')
        else:
            print()
            for uid in self.uav_ids:
                state = self.uav_states.get(uid, '-')
                cov = self.uav_coverage.get(uid, 0.0)
                print(f'{uid} | {state:<12} | coverage: {cov * 100:.1f}%')

        print('\nCommands: start | stop | exit')

    # ===== Operator Interface =====

    def send_start(self):
        if self.mission_state != 'IDLE':
            print('Mission already running')
            return
        self.mission_state = 'RUNNING'
        self.uav_states = {uid: 'IDLE' for uid in self.uav_ids}
        self.uav_coverage = {uid: 0.0 for uid in self.uav_ids}
        self._start_pub.publish(Empty())
        self.get_logger().debug('START sent')

    def send_stop(self):
        if self.mission_state == 'IDLE':
            print('Mission not running')
            return
        self.mission_state = 'STOPPED'
        self._stop_pub.publish(Empty())
        self.get_logger().debug('STOP sent')


def _safe_spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    spin_thread = threading.Thread(target=_safe_spin, args=(node,), daemon=True)
    spin_thread.start()

    node._refresh_dashboard()

    try:
        while rclpy.ok():
            cmd = input('>> ').strip().lower()
            if cmd == 'start':
                node.send_start()
            elif cmd == 'stop':
                node.send_stop()
            elif cmd == 'exit':
                break
            else:
                print('Unknown command')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
