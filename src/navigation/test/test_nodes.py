# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for navigation nodes."""

import math

from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry, Path
from navigation.astar_navigation_node import AStarNavigationNode
from navigation.path_executor import PathExecutorNode
from navigation.world_model import WorldModelNode
import pytest
import rclpy
from sar_msgs.msg import FSMEvent
from sar_msgs.srv import GetOccupancyGrid
from std_msgs.msg import Empty


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


def _make_odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    return msg


def _make_pose(x, y):
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.orientation.w = 1.0
    return p


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


def test_grid_initializes_as_unknown():
    """All cells start as unknown (-1) before any sensor data arrives."""
    uut = WorldModelNode()
    helper = rclpy.create_node('test_grid_unknown_helper')
    client = helper.create_client(GetOccupancyGrid, '/x1/world_model/get_grid')

    _spin(uut, helper, 10)
    assert client.wait_for_service(timeout_sec=1.0)

    future = client.call_async(GetOccupancyGrid.Request())
    _spin(uut, helper, 30)

    assert future.done()
    grid_flat = future.result().grid
    assert all(v == -1 for v in grid_flat)

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

    # publish x2 at world (0, 0) → grid cell (10, 10), index 10*21+10 = 220
    odom_pub.publish(_make_odom(0.0, 0.0))
    _spin(uut, helper, 20)

    future = client.call_async(GetOccupancyGrid.Request())
    _spin(uut, helper, 30)

    assert future.done()
    assert future.result().grid[220] == 1

    uut.destroy_node()
    helper.destroy_node()


def test_dynamic_obstacle_clears_to_unknown():
    """When a UAV moves, its old grid position reverts to unknown (-1)."""
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
    assert grid_flat[220] == -1, 'old position should revert to unknown'
    assert grid_flat[44] == 1, 'new position should be marked occupied'

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

    other_pub.publish(_make_odom(5.0, 5.0))
    _spin(uut, helper, 20)

    assert not any(e.event == 'COLLISION_RISK' for e in events)

    uut.destroy_node()
    helper.destroy_node()


# ===== AStarNavigationNode =====

def test_astar_waypoints_stored_on_receive():
    """Receiving a PoseArray stores waypoints and resets index to 0."""
    uut = AStarNavigationNode()
    helper = rclpy.create_node('test_astar_store_helper')

    qos_transient = rclpy.qos.QoSProfile(
        depth=1,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    )
    wp_pub = helper.create_publisher(PoseArray, '/x1/nav/waypoints', qos_transient)

    # give the node a pose so it doesn't bail out early
    uut.current_pose = (-5.0, -5.0)

    _spin(uut, helper, 10)

    msg = PoseArray()
    msg.poses = [_make_pose(0.0, 0.0), _make_pose(3.0, 3.0)]
    wp_pub.publish(msg)
    _spin(uut, helper, 20)

    assert len(uut.waypoints) == 2
    assert uut.waypoint_index == 0
    assert uut.waypoints[0] == (0.0, 0.0)
    assert uut.waypoints[1] == (3.0, 3.0)

    uut.destroy_node()
    helper.destroy_node()


def test_astar_advances_on_waypoint_reached():
    """Waypoint index increments when reached_coverage_waypoint fires."""
    uut = AStarNavigationNode()

    # set state directly to avoid needing a running world model
    uut.waypoints = [(0.0, 0.0), (3.0, 3.0)]
    uut.waypoint_index = 0
    uut.current_pose = (-5.0, -5.0)

    uut._on_waypoint_reached(Empty())

    assert uut.waypoint_index == 1

    uut.destroy_node()


def test_astar_clears_waypoints_when_sequence_complete():
    """Waypoint list is cleared after the last reached signal."""
    uut = AStarNavigationNode()

    uut.waypoints = [(0.0, 0.0)]
    uut.waypoint_index = 0
    uut.current_pose = (-5.0, -5.0)

    uut._on_waypoint_reached(Empty())

    assert len(uut.waypoints) == 0
    assert uut.current_path is None

    uut.destroy_node()


# ===== PathExecutorNode =====

def _make_path(points):
    """Build a nav_msgs/Path from a list of (x, y) tuples."""
    from geometry_msgs.msg import PoseStamped
    msg = Path()
    for x, y in points:
        ps = PoseStamped()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def test_executor_stores_path_on_receive():
    """Receiving a Path message stores cells and resets index to 0."""
    uut = PathExecutorNode()
    helper = rclpy.create_node('test_exec_store_helper')

    qos_transient = rclpy.qos.QoSProfile(
        depth=1,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    )
    path_pub = helper.create_publisher(Path, '/x1/nav/planned_path', qos_transient)

    # place UAV far from path so the move timer doesn't advance current_index
    uut.uav_x = -50.0
    uut.uav_y = -50.0

    _spin(uut, helper, 10)

    path_pub.publish(_make_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]))
    _spin(uut, helper, 20)

    assert len(uut.current_path) == 3
    assert uut.current_index == 0
    assert uut.current_path[2] == (2.0, 0.0)

    uut.destroy_node()
    helper.destroy_node()


def test_executor_home_captured_on_first_odom():
    """Home position is set from the first odometry message."""
    uut = PathExecutorNode()
    helper = rclpy.create_node('test_exec_home_helper')
    odom_pub = helper.create_publisher(Odometry, '/x1/state/odom', 10)

    _spin(uut, helper, 10)
    assert uut.home_x is None

    odom_pub.publish(_make_odom(3.0, 4.0))
    _spin(uut, helper, 10)

    assert uut.home_x == pytest.approx(3.0)
    assert uut.home_y == pytest.approx(4.0)

    # second message must NOT change home
    odom_pub.publish(_make_odom(10.0, 10.0))
    _spin(uut, helper, 10)

    assert uut.home_x == pytest.approx(3.0)

    uut.destroy_node()
    helper.destroy_node()


def test_executor_stop_clears_path():
    """Publishing to /mission/stop clears the active path."""
    uut = PathExecutorNode()
    helper = rclpy.create_node('test_exec_stop_helper')
    stop_pub = helper.create_publisher(Empty, '/mission/stop', 10)

    uut.current_path = [(0.0, 0.0), (1.0, 0.0)]
    uut.current_index = 1

    _spin(uut, helper, 10)
    stop_pub.publish(Empty())
    _spin(uut, helper, 20)

    assert len(uut.current_path) == 0
    assert uut.current_index == 0
    assert not uut.is_returning

    uut.destroy_node()
    helper.destroy_node()


def test_executor_go_home_sets_returning_flag():
    """go_home() sets is_returning and publishes a single home waypoint."""
    uut = PathExecutorNode()
    helper = rclpy.create_node('test_exec_gohome_helper')

    qos_transient = rclpy.qos.QoSProfile(
        depth=1,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    )
    received = []
    helper.create_subscription(PoseArray, '/x1/nav/waypoints', received.append, qos_transient)

    uut.home_x = 1.0
    uut.home_y = 2.0

    _spin(uut, helper, 10)
    uut.go_home()
    _spin(uut, helper, 20)

    assert uut.is_returning
    assert len(received) == 1
    assert received[0].poses[0].position.x == pytest.approx(1.0)
    assert received[0].poses[0].position.y == pytest.approx(2.0)

    uut.destroy_node()
    helper.destroy_node()


def test_executor_home_reached_event_on_return_complete():
    """HOME_REACHED FSM event is published when the return path finishes."""
    uut = PathExecutorNode()
    helper = rclpy.create_node('test_exec_homereach_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    # warm-up so publisher/subscriber connection is established before we need the event
    _spin(uut, helper, 10)

    # simulate: already on return path, sitting exactly at the last cell
    uut.is_returning = True
    uut.home_x = 0.0
    uut.home_y = 0.0
    uut.current_path = [(0.05, 0.0)]  # one cell, within threshold
    uut.current_index = 0
    uut.uav_x = 0.0
    uut.uav_y = 0.0
    uut.path_cells_total = 1

    _spin(uut, helper, 30)

    assert any(e.event == 'HOME_REACHED' for e in events)
    assert not uut.is_returning
    assert len(uut.current_path) == 0

    uut.destroy_node()
    helper.destroy_node()
