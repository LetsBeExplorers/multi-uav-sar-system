# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for uav_platform nodes."""

from geometry_msgs.msg import Twist
import pytest
import rclpy
from sar_msgs.msg import DriverHealth, FSMEvent
from uav_platform.platform_interface import PlatformInterface


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


# ===== PlatformInterface =====

def test_velocity_clamping():
    """Oversized linear velocities are clamped to the configured limits."""
    uut = PlatformInterface()
    helper = rclpy.create_node('test_clamp_helper')
    received = []
    helper.create_subscription(Twist, '/x1/driver/cmd_vel', received.append, 10)
    pub = helper.create_publisher(Twist, '/x1/platform/cmd_vel', 10)

    _spin(uut, helper, 10)

    msg = Twist()
    msg.linear.x = 10.0   # max_linear = 3.0
    msg.linear.z = 5.0    # max_vertical = 2.0
    pub.publish(msg)
    _spin(uut, helper, 20)

    assert len(received) > 0
    assert abs(received[-1].linear.x - 3.0) < 1e-6
    assert abs(received[-1].linear.z - 2.0) < 1e-6

    uut.destroy_node()
    helper.destroy_node()


def test_negative_velocity_clamping():
    """Negative oversized velocities are clamped in the negative direction."""
    uut = PlatformInterface()
    helper = rclpy.create_node('test_neg_clamp_helper')
    received = []
    helper.create_subscription(Twist, '/x1/driver/cmd_vel', received.append, 10)
    pub = helper.create_publisher(Twist, '/x1/platform/cmd_vel', 10)

    _spin(uut, helper, 10)

    msg = Twist()
    msg.linear.x = -99.0
    pub.publish(msg)
    _spin(uut, helper, 20)

    assert len(received) > 0
    assert abs(received[-1].linear.x - (-3.0)) < 1e-6

    uut.destroy_node()
    helper.destroy_node()


def test_low_battery_event():
    """Battery below threshold triggers a LOW_BATTERY FSM event."""
    uut = PlatformInterface()
    helper = rclpy.create_node('test_battery_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)
    pub = helper.create_publisher(DriverHealth, '/x1/driver/health', 10)

    _spin(uut, helper, 10)

    msg = DriverHealth()
    msg.status = 'OK'
    msg.battery = 10.0   # below threshold of 20.0
    msg.timestamp = 0.0
    pub.publish(msg)
    _spin(uut, helper, 20)

    assert any(e.event == 'LOW_BATTERY' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_comms_loss_event():
    """COMMS_LOSS health status triggers a COMMS_LOSS FSM event."""
    uut = PlatformInterface()
    helper = rclpy.create_node('test_comms_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)
    pub = helper.create_publisher(DriverHealth, '/x1/driver/health', 10)

    _spin(uut, helper, 10)

    msg = DriverHealth()
    msg.status = 'COMMS_LOSS'
    msg.battery = 100.0
    msg.timestamp = 0.0
    pub.publish(msg)
    _spin(uut, helper, 20)

    assert any(e.event == 'COMMS_LOSS' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_healthy_health_no_events():
    """Healthy status with full battery produces no FSM events."""
    uut = PlatformInterface()
    helper = rclpy.create_node('test_healthy_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)
    pub = helper.create_publisher(DriverHealth, '/x1/driver/health', 10)

    _spin(uut, helper, 10)

    msg = DriverHealth()
    msg.status = 'OK'
    msg.battery = 100.0
    msg.timestamp = 0.0
    pub.publish(msg)
    _spin(uut, helper, 20)

    assert len(events) == 0

    uut.destroy_node()
    helper.destroy_node()
