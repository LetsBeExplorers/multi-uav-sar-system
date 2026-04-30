import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sar_msgs.msg import MissionCoverage, UAVState, Alert, DetectionEvent
from std_msgs.msg import Empty, String


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ===== Parameters =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_uavs', 3),
                ('threshold', 0.90),
            ]
        )

        num_uavs = self.get_parameter('num_uavs').value
        self.threshold = self.get_parameter('threshold').value

        # ===== State =====
        self.uav_ids = [f'x{i + 1}' for i in range(num_uavs)]
        self.mission_state = 'IDLE'
        self.uav_states = {uid: 'IDLE' for uid in self.uav_ids}
        self.uav_coverage = {uid: 0.0 for uid in self.uav_ids}
        self.uav_area = {uid: (0.0, 0.0) for uid in self.uav_ids}  # (covered, assigned) m²
        self.alert_log = []
        self.target_goal = 0
        self.targets_found = 0
        self.confirmed_targets = []

        # ===== Publishers =====
        self._start_pub = self.create_publisher(Empty, '/mission/start', 10)
        self._stop_pub = self.create_publisher(Empty, '/mission/stop', 10)
        self._halt_pub = self.create_publisher(Empty, '/mission/halt', 10)
        self._coverage_pub = self.create_publisher(MissionCoverage, '/mission/coverage', 10)

        # ===== Subscribers =====
        self.create_subscription(UAVState, '/uav/state', self._on_uav_state_change, 10)
        self.create_subscription(String, '/mission/status', self._on_status, 10)
        self.create_subscription(Alert, '/alerts', self._on_alert, 10)
        self.create_subscription(DetectionEvent, '/targets/confirmed', self._on_target_confirmed, 10)

    # ===== State Tracking =====

    def _on_uav_state_change(self, msg):
        if msg.uav_id in self.uav_states:
            self.uav_states[msg.uav_id] = msg.state

        if (
            self.target_goal == 0 and
            all(s == 'IDLE' for s in self.uav_states.values()) and
            self.mission_state == 'RUNNING'
        ):
            self._complete_mission()

        self._refresh_dashboard()

    def _on_status(self, msg):
        text = msg.data

        if 'PROGRESS' in text:
            try:
                # Format: "[x1] PROGRESS: 50/100 AREA: 200.0/400.0"
                uav_id = text.split(']')[0].lstrip('[')
                if 'AREA:' in text:
                    area = text.split('AREA:')[1].strip()
                    covered, assigned = area.split('/')
                    covered_f = float(covered)
                    assigned_f = float(assigned)
                    self.uav_area[uav_id] = (covered_f, assigned_f)
                    # grid coverage (monotonic) — route progress resets each phase
                    if assigned_f > 0:
                        self.uav_coverage[uav_id] = covered_f / assigned_f
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

    # ===== Alerts =====

    def _on_alert(self, msg):
        self.alert_log.append(msg)

        # keep only last 5 alerts (prevents spam explosion)
        self.alert_log = self.alert_log[-5:]

        self._refresh_dashboard()

    def _color(self, level):
        if level == 'CRITICAL':
            return '\033[91m'  # red
        elif level == 'WARNING':
            return '\033[93m'  # yellow
        else:
            return '\033[0m'


    def _on_target_confirmed(self, msg):
        if self.mission_state != 'RUNNING':
            return
            
        new_target = (msg.x, msg.y)

        # prevent duplicates
        for (x, y) in self.confirmed_targets:
            if abs(x - new_target[0]) < 1.0 and abs(y - new_target[1]) < 1.0:
                return

        self.confirmed_targets.append(new_target)
        self.targets_found += 1

        if self.targets_found >= self.target_goal:
            self._complete_mission()

    # ===== Dashboard =====

    def _refresh_dashboard(self):
        print('\033[H\033[J', end='')
        print('=== MISSION STATUS ===')
        print(f'Mission: {self.mission_state}')
        if self.mission_state in ['RUNNING', 'COMPLETE']:
            print(f'Targets: {self.targets_found}/{self.target_goal}')

        if self.mission_state == 'IDLE':
            print('\nWaiting for start command...')
        else:
            print()
            for uid in self.uav_ids:
                state = self.uav_states.get(uid, '-')
                cov = self.uav_coverage.get(uid, 0.0)
                covered, assigned = self.uav_area.get(uid, (0.0, 0.0))
                print(
                    f'{uid} | {state:<12} | coverage: {cov * 100:5.1f}% '
                    f'| area: {covered:6.1f} / {assigned:6.1f} m²'
                )

        print('\n=== ALERTS ===')

        if not self.alert_log:
            print('None')
        else:
            for alert in reversed(self.alert_log):
                color = self._color(alert.level)
                print(
                    f'{color}[{alert.uav_id}] {alert.level:<8} | {alert.type:<18} | {alert.message}\033[0m'
                )

        print('\nCommands: start | end | stop | exit')

    # ===== Operator Interface =====

    def send_start(self, target_goal=0):
        if self.mission_state == 'RUNNING':
            print('Mission already running')
            return

        self.target_goal = target_goal
        self.targets_found = 0
        self.confirmed_targets = []

        print(f'Starting mission with target goal: {target_goal}')

        self._wait_for_subscribers(self._start_pub)
        self.mission_state = 'RUNNING'
        self.uav_states = {uid: 'IDLE' for uid in self.uav_ids}
        self.uav_coverage = {uid: 0.0 for uid in self.uav_ids}
        self.uav_area = {uid: (0.0, 0.0) for uid in self.uav_ids}

        self._start_pub.publish(Empty())

    def send_stop(self):
        if self.mission_state == 'IDLE':
            print('Mission not running')
            return
        self._wait_for_subscribers(self._stop_pub)
        self.mission_state = 'STOPPED'
        self._stop_pub.publish(Empty())
        self.get_logger().debug('STOP sent')

    def send_halt(self):
        if self.mission_state == 'IDLE':
            print('Mission not running')
            return

        self._wait_for_subscribers(self._halt_pub)
        self.mission_state = 'HALTED'
        self._halt_pub.publish(Empty())
        self.get_logger().debug('HALT sent')

    def _wait_for_subscribers(self, pub, timeout=5.0):
        # Block until all UAVs are subscribed to `pub` or timeout expires.
        deadline = time.time() + timeout
        while pub.get_subscription_count() < len(self.uav_ids):
            if time.time() > deadline:
                return
            time.sleep(0.1)

    def _complete_mission(self):
        if self.mission_state == 'COMPLETE':
            return

        self.mission_state = 'COMPLETE'
        self._wait_for_subscribers(self._stop_pub)
        self._stop_pub.publish(Empty())


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
            parts = cmd.split()
            if not parts:
                continue

            if parts[0] == 'start':
                if len(parts) > 1:
                    try:
                        target_goal = int(parts[1])
                    except ValueError:
                        print('Invalid number of targets')
                        continue
                else:
                    target_goal = 0  # default

                node.send_start(target_goal)
            elif cmd == 'end':
                node.send_stop()
            elif cmd == 'stop':
                node.send_halt()
            elif cmd == 'exit':
                break
            else:
                print('Unknown command')
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
