# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for mission_manager nodes."""

from unittest.mock import patch

from mission_manager.mission_manager import MissionManager
import pytest
import rclpy
from sar_msgs.msg import MissionCoverage, UAVState
from std_msgs.msg import Empty, String


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize and shut down rclpy for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(a, b, count=20):
    """Alternate spinning two nodes to exchange messages."""
    for _ in range(count):
        rclpy.spin_once(a, timeout_sec=0.05)
        rclpy.spin_once(b, timeout_sec=0.05)


# ===== MissionManager =====

def test_progress_updates_coverage():
    """PROGRESS status string updates the coverage ratio for the named UAV."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        helper = rclpy.create_node('test_progress_helper')
        coverage_msgs = []
        helper.create_subscription(
            MissionCoverage, '/mission/coverage', coverage_msgs.append, 10)
        pub = helper.create_publisher(String, '/mission/status', 10)

        _spin(uut, helper, 10)

        msg = String()
        msg.data = '[x1] PROGRESS: 50/100'
        pub.publish(msg)
        _spin(uut, helper, 20)

        assert len(coverage_msgs) > 0
        idx = coverage_msgs[-1].uav_ids.index('x1')
        assert abs(coverage_msgs[-1].coverage_ratios[idx] - 0.5) < 1e-6

        uut.destroy_node()
        helper.destroy_node()


def test_all_complete_flag():
    """all_complete is True when every UAV exceeds the coverage threshold."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        helper = rclpy.create_node('test_complete_helper')
        coverage_msgs = []
        helper.create_subscription(
            MissionCoverage, '/mission/coverage', coverage_msgs.append, 10)
        pub = helper.create_publisher(String, '/mission/status', 10)

        _spin(uut, helper, 10)

        for uid in ['x1', 'x2', 'x3']:
            msg = String()
            msg.data = f'[{uid}] PROGRESS: 96/100'
            pub.publish(msg)
            _spin(uut, helper, 10)

        assert len(coverage_msgs) > 0
        assert coverage_msgs[-1].all_complete is True

        uut.destroy_node()
        helper.destroy_node()


def test_below_threshold_not_complete():
    """all_complete is False when a UAV is below the coverage threshold."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        helper = rclpy.create_node('test_incomplete_helper')
        coverage_msgs = []
        helper.create_subscription(
            MissionCoverage, '/mission/coverage', coverage_msgs.append, 10)
        pub = helper.create_publisher(String, '/mission/status', 10)

        _spin(uut, helper, 10)

        msg = String()
        msg.data = '[x1] PROGRESS: 50/100'
        pub.publish(msg)
        _spin(uut, helper, 20)

        assert len(coverage_msgs) > 0
        assert coverage_msgs[-1].all_complete is False

        uut.destroy_node()
        helper.destroy_node()


def test_send_start_publishes():
    """send_start publishes an Empty message on /mission/start."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        helper = rclpy.create_node('test_start_helper')
        starts = []
        helper.create_subscription(Empty, '/mission/start', starts.append, 10)

        _spin(uut, helper, 10)
        uut.send_start()
        _spin(uut, helper, 20)

        assert len(starts) > 0

        uut.destroy_node()
        helper.destroy_node()


def test_send_stop_publishes():
    """send_stop publishes an Empty message on /mission/stop."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        uut.mission_state = 'RUNNING'
        helper = rclpy.create_node('test_stop_helper')
        stops = []
        helper.create_subscription(Empty, '/mission/stop', stops.append, 10)

        _spin(uut, helper, 10)
        uut.send_stop()
        _spin(uut, helper, 20)

        assert len(stops) > 0

        uut.destroy_node()
        helper.destroy_node()


def test_all_uavs_idle_sets_complete():
    """Mission state becomes COMPLETE when all UAVs report IDLE while running."""
    with patch.object(MissionManager, '_refresh_dashboard'):
        uut = MissionManager()
        uut.mission_state = 'RUNNING'
        helper = rclpy.create_node('test_idle_helper')
        pub = helper.create_publisher(UAVState, '/uav/state', 10)

        _spin(uut, helper, 10)

        for uid in ['x1', 'x2', 'x3']:
            msg = UAVState()
            msg.uav_id = uid
            msg.state = 'IDLE'
            msg.previous_state = 'RETURNING'
            msg.timestamp = 0.0
            pub.publish(msg)
            _spin(uut, helper, 10)

        assert uut.mission_state == 'COMPLETE'

        uut.destroy_node()
        helper.destroy_node()
