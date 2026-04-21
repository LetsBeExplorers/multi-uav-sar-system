# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for navigation nodes."""

import math

from nav_msgs.msg import Odometry
from navigation.world_model import WorldModelNode
import pytest
import rclpy
from sar_msgs.msg import FSMEvent
from sar_msgs.srv import GetOccupancyGrid


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


def _spin_one(node, count=20):
    for _ in range(count):
        rclpy.spin_once(node, timeout_sec=0.05)


def _make_odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    return msg


# ===== WorldModelNode =====

def test_grid_service_returns_correct_dimensions():
    """Service returns the expected width, height, resolution, and flat grid size."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_grid_dims_helper')
    client = helper.create_client(GetOccupancyGrid, '/x1/world_model/get_grid')

    _spin(uut, helper, 10)
    assert client.wait_for_service(timeout_sec=1.0)

    future = client.call_async(GetOccupancyGrid.Request())
    _spin(uut, helper, 30)

    assert future.done()
    resp = future.result()
    assert resp.width == 21
    assert resp.height == 21
    assert len(resp.grid) == 21 * 21
    assert resp.resolution == 1.0
    assert resp.origin_x == -10.0
    assert resp.origin_y == -10.0

    uut.destroy_node()
    helper.destroy_node()


def test_dynamic_obstacle_marks_grid():
    """Publishing another UAV's pose causes those cells to be marked occupied."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_dyn_obs_helper')
    odom_pub = helper.create_publisher(Odometry, '/x2/state/odom', 10)
    client = helper.create_client(GetOccupancyGrid, '/x1/world_model/get_grid')

    _spin(uut, helper, 10)
    assert client.wait_for_service(timeout_sec=1.0)

    # publish x2 at world (0, 0) → grid cell (10, 10)
    odom_pub.publish(_make_odom(0.0, 0.0))
    _spin(uut, helper, 20)

    future = client.call_async(GetOccupancyGrid.Request())
    _spin(uut, helper, 30)

    assert future.done()
    grid_flat = future.result().grid
    # cell (10, 10) = row 10, col 10 → index 10*21 + 10 = 220
    assert grid_flat[220] == 1

    uut.destroy_node()
    helper.destroy_node()


def test_dynamic_obstacle_clears_old_position():
    """When a UAV moves, its old grid position is cleared."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_dyn_clear_helper')
    odom_pub = helper.create_publisher(Odometry, '/x2/state/odom', 10)
    client = helper.create_client(GetOccupancyGrid, '/x1/world_model/get_grid')

    _spin(uut, helper, 10)
    assert client.wait_for_service(timeout_sec=1.0)

    # place x2 at (0, 0) → cell (10, 10), index 220
    odom_pub.publish(_make_odom(0.0, 0.0))
    _spin(uut, helper, 20)

    # move x2 far away to (-8, -8) → cell (2, 2), index 2*21+2 = 44
    odom_pub.publish(_make_odom(-8.0, -8.0))
    _spin(uut, helper, 20)

    future = client.call_async(GetOccupancyGrid.Request())
    _spin(uut, helper, 30)

    assert future.done()
    grid_flat = future.result().grid
    assert grid_flat[220] == 0, 'old position should be cleared'
    assert grid_flat[44] == 1, 'new position should be marked'

    uut.destroy_node()
    helper.destroy_node()


def test_collision_risk_event_published():
    """COLLISION_RISK FSM event is published when another UAV comes within threshold."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_collision_helper')
    own_pub = helper.create_publisher(Odometry, '/x1/state/odom', 10)
    other_pub = helper.create_publisher(Odometry, '/x2/state/odom', 10)
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)

    # set own pose at (0, 0)
    own_pub.publish(_make_odom(0.0, 0.0))
    _spin(uut, helper, 10)

    # place x2 at (0.3, 0) — within collision_threshold of 0.5m
    other_pub.publish(_make_odom(0.3, 0.0))
    _spin(uut, helper, 20)

    assert any(e.event == 'COLLISION_RISK' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_no_collision_event_when_far():
    """No COLLISION_RISK event when UAVs are well separated."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_no_collision_helper')
    own_pub = helper.create_publisher(Odometry, '/x1/state/odom', 10)
    other_pub = helper.create_publisher(Odometry, '/x2/state/odom', 10)
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)

    own_pub.publish(_make_odom(0.0, 0.0))
    _spin(uut, helper, 10)

    # place x2 at (5, 5) — well outside threshold
    other_pub.publish(_make_odom(5.0, 5.0))
    _spin(uut, helper, 20)

    assert not any(e.event == 'COLLISION_RISK' for e in events)

    uut.destroy_node()
    helper.destroy_node()
