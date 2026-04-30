import threading
import csv
import os
import time
import random
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sar_msgs.msg import MissionCoverage, UAVState, Alert, DetectionEvent, UAVCoverage
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
        self.detection_log = []
        self.failures = []
        self.is_test = 'PYTEST_CURRENT_TEST' in os.environ

        # ===== Publishers =====
        self._start_pub = self.create_publisher(Empty, '/mission/start', 10)
        self._stop_pub = self.create_publisher(Empty, '/mission/stop', 10)
        self._end_pub = self.create_publisher(Empty, '/mission/end', 10)
        self._complete_pub = self.create_publisher(Empty, '/mission/complete', 10)
        self._coverage_pub = self.create_publisher(MissionCoverage, '/mission/coverage', 10)
        self._targets_pub = self.create_publisher(DetectionEvent, '/mission/targets', 10) # temporary

        # ===== Subscribers =====
        self.create_subscription(UAVState, '/uav/state', self._on_uav_state_change, 10)
        self.create_subscription(UAVCoverage, '/uav/coverage', self._on_coverage_msg, 10)
        self.create_subscription(Alert, '/alerts', self._on_alert, 10)
        self.create_subscription(DetectionEvent, '/targets/confirmed', self._on_target_confirmed, 10)

        # ===== Mission timing =====
        self.mission_start_time = None
        self._summary_written = False

        # ===== Results Logging =====
        if not self.is_test:
            results_dir = "results"
            os.makedirs(results_dir, exist_ok=True)

            now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.results_file_path = os.path.join("results", f"mission_{now}.csv")

            self.csv_file = open(self.results_file_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Event log header
            self.csv_writer.writerow(['time', 'uav_id', 'event_type'])
        else:
            self.csv_file = None
            self.csv_writer = None


    # ===== ROS Callbacks =====

    def _on_uav_state_change(self, msg):
        prev = self.uav_states.get(msg.uav_id)

        if msg.uav_id in self.uav_states:
            self.uav_states[msg.uav_id] = msg.state

        if prev != msg.state:
            self._refresh_dashboard()

        self._check_mission_complete()

    def _on_coverage_msg(self, msg):
        if self.mission_state == 'IDLE':
            return
        prev = self.uav_coverage.get(msg.uav_id, None)

        self._update_coverage(
            msg.uav_id,
            msg.covered_area,
            msg.assigned_area
        )

        new = self.uav_coverage.get(msg.uav_id, None)

        if prev != new:
            self._refresh_dashboard()

    def _on_alert(self, msg):
        # dashboard display
        self.alert_log.append(msg)
        self.alert_log = self.alert_log[-5:]

        # track failures
        if msg.level in ('WARNING', 'CRITICAL'):
            t = self.get_clock().now().nanoseconds / 1e9

            self.failures.append({
                "uav_id": msg.uav_id,
                "type": msg.type,
                "time": t
            })

            self._log_failure(msg, t)

        self._refresh_dashboard()

    def _on_target_confirmed(self, msg):
        if self.mission_state != 'RUNNING':
            return
            
        t = msg.timestamp
        new_target = (msg.x, msg.y, t)

        # prevent duplicates
        for (x, y, *_ ) in self.confirmed_targets:
            if abs(x - new_target[0]) < 1.0 and abs(y - new_target[1]) < 1.0:
                return

        dt = t - self.mission_start_time
        self.detection_log.append(dt)

        self.confirmed_targets.append(new_target)
        self.targets_found += 1
        self._check_mission_complete()


    # ===== Core Logic =====

    def _check_mission_complete(self):
        if self.mission_state != 'RUNNING':
            return

        # target-based completion
        if self.target_goal > 0 and self.targets_found >= self.target_goal:
            self._complete_mission()
            return

        # coverage-based completion
        if (
            self.target_goal == 0 and
            all(s == 'IDLE' for s in self.uav_states.values())
        ):
            self._complete_mission()

    def _reset_uav_state(self):
        self.uav_states = {uid: 'IDLE' for uid in self.uav_ids}
        self.uav_coverage = {uid: 0.0 for uid in self.uav_ids}
        self.uav_area = {uid: (0.0, 0.0) for uid in self.uav_ids}

    def _update_coverage(self, uav_id, covered, assigned):
        self.uav_area[uav_id] = (covered, assigned)
        if assigned > 0:
            self.uav_coverage[uav_id] = covered / assigned
            self._publish_coverage()

    def _publish_coverage(self):
        msg = MissionCoverage()
        msg.uav_ids = self.uav_ids
        msg.coverage_ratios = [self.uav_coverage.get(uid, 0.0) for uid in self.uav_ids]
        msg.all_complete = all(r >= self.threshold for r in msg.coverage_ratios)
        msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self._coverage_pub.publish(msg)

    def _complete_mission(self):
        if self.mission_state == 'COMPLETE':
            return

        self.mission_state = 'COMPLETE'
        self._log_mission_summary(success=True)
        self._wait_for_subscribers(self._complete_pub)
        self._complete_pub.publish(Empty())


    # ===== Logging =====

    def _log_failure(self, msg, t):
        if self.csv_writer:
            self.csv_writer.writerow([t, msg.uav_id, msg.type])
            self.csv_file.flush()

    def _log_mission_summary(self, success=False):
        if self._summary_written or not self.csv_writer:
            return
        self._summary_written = True

        t_end = self.get_clock().now().nanoseconds / 1e9
        mission_time = round(t_end - self.mission_start_time, 2) if self.mission_start_time else -1

        # Per-UAV coverage
        coverages = {uid: round(self.uav_coverage.get(uid, 0.0), 4) for uid in self.uav_ids}
        avg_coverage = round(sum(coverages.values()) / len(coverages), 4) if coverages else 0.0

        collision_count = sum(
            1 for f in self.failures
            if f.get('type', '').upper() == 'HARD_COLLISION'
        )

        collision_risk_count = sum(
            1 for f in self.failures
            if f.get('type', '').upper() == 'COLLISION_RISK'
        )
        
        path_fail_count = sum(1 for f in self.failures if f.get('type', '').upper() == 'REPLAN_FAIL')

        if self.detection_log:
            avg_dt = round(sum(self.detection_log) / len(self.detection_log), 2)
        else:
            avg_dt = -1

        # Write a blank separator line then the summary block
        self.csv_writer.writerow([])
        self.csv_writer.writerow(['--- MISSION SUMMARY ---'])
        self.csv_writer.writerow(['success', 'mission_time_s', 'targets_found',
                                  'avg_detection_time',
                                  'avg_coverage', 'collisions', 'collision_risks', 'path_failures',
                                  *[f'coverage_{uid}' for uid in self.uav_ids]])
        self.csv_writer.writerow([
            int(success),
            mission_time,
            self.targets_found,
            avg_dt,
            avg_coverage,
            collision_count,
            collision_risk_count,
            path_fail_count,
            *[coverages[uid] for uid in self.uav_ids],
        ])
        self.csv_file.flush()

    # ===== UI =====

    def _color(self, level):
        if level == 'CRITICAL':
            return '\033[91m'  # red
        elif level == 'WARNING':
            return '\033[93m'  # yellow
        else:
            return '\033[0m'

    def _refresh_dashboard(self):
        if self.is_test:
            return

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
        self.targets = []

        for _ in range(target_goal):
            x = random.uniform(-8, 8)
            y = random.uniform(-8, 8)
            self.targets.append((x, y))

        self._wait_for_subscribers(self._targets_pub)

        for (x, y) in self.targets:
            msg = DetectionEvent()
            msg.uav_id = 'mission'
            msg.x = x
            msg.y = y
            msg.confidence = 1.0
            msg.timestamp = self.get_clock().now().nanoseconds / 1e9

            self._targets_pub.publish(msg)

        self.targets_found = 0
        self.confirmed_targets = []
        self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
        self._summary_written = False

        print(f'Starting mission with target goal: {target_goal}')

        self._wait_for_subscribers(self._start_pub)

        self.mission_state = 'RUNNING'

        self._reset_uav_state()
        self._start_pub.publish(Empty())

    def send_end(self):
        if self.mission_state == 'IDLE':
            print('Mission not running')
            return

        self._wait_for_subscribers(self._end_pub)
        self.mission_state = 'ENDED'
        self._end_pub.publish(Empty())

    def send_stop(self):
        if self.mission_state == 'IDLE':
            print('Mission not running')
            return

        self._wait_for_subscribers(self._stop_pub)
        self.mission_state = 'STOPPED'
        self._stop_pub.publish(Empty())

    def _wait_for_subscribers(self, pub, timeout=5.0):
        if self.is_test:
            return

        # Block until all UAVs are subscribed to `pub` or timeout expires.
        deadline = time.time() + timeout
        while pub.get_subscription_count() < len(self.uav_ids):
            if time.time() > deadline:
                return
            time.sleep(0.1)

    def destroy_node(self):
        if self.mission_state in ('RUNNING', 'ENDED', 'STOPPED'):
            self._log_mission_summary(success=(self.mission_state == 'COMPLETE'))
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


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
                node.send_end()
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