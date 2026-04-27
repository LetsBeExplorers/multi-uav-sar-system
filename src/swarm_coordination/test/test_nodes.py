# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for swarm coordination nodes."""

import pytest
import rclpy
from geometry_msgs.msg import PoseArray
from sar_msgs.msg import FSMEvent, MissionCoverage, UAVState
from std_msgs.msg import Empty, String
from swarm_coordination.swarm_coordinator import SwarmCoordinator, _lawnmower
from swarm_coordination.uav_state_manager import UAVStateManager


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(a, b, count=20):
    for _ in range(count):
        rclpy.spin_once(a, timeout_sec=0.05)
        rclpy.spin_once(b, timeout_sec=0.05)


def _make_fsm(uav_id='x1', threshold=0.95):
    return UAVStateManager()


# ===== Initial state =====

def test_initial_state_is_idle():
    uut = _make_fsm()
    assert uut.current_state == 'IDLE'
    uut.destroy_node()


# ===== Normal mission flow =====

def test_mission_start_from_idle():
    uut = _make_fsm()
    uut._handle_event('MISSION_START')
    assert uut.current_state == 'SEARCHING'
    uut.destroy_node()


def test_region_complete_below_threshold_goes_to_refining():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('REGION_COMPLETE', value=0.80)
    assert uut.current_state == 'REFINING'
    uut.destroy_node()


def test_region_complete_at_threshold_goes_to_assisting():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('REGION_COMPLETE', value=0.95)
    assert uut.current_state == 'ASSISTING'
    uut.destroy_node()


def test_region_complete_above_threshold_goes_to_assisting():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('REGION_COMPLETE', value=1.0)
    assert uut.current_state == 'ASSISTING'
    uut.destroy_node()


def test_refinement_complete_goes_to_assisting():
    uut = _make_fsm()
    uut.current_state = 'REFINING'
    # x1 is the nearest helper for x2 (the only target), so x1 gets paired
    uut.coverage_map = {'x1': 0.95, 'x2': 0.50, 'x3': 0.95}
    uut._handle_event('REFINEMENT_COMPLETE')
    assert uut.current_state == 'ASSISTING'
    uut.destroy_node()


def test_all_drones_done_from_assisting_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('ALL_DRONES_DONE')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_all_drones_done_from_searching_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('ALL_DRONES_DONE')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_all_drones_done_from_refining_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'REFINING'
    uut._handle_event('ALL_DRONES_DONE')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_assist_complete_stays_assisting():
    """ASSIST_COMPLETE re-emits ASSISTING so the coordinator picks a new target."""
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    # x2 still below threshold; x1 is the nearest unpaired helper
    uut.coverage_map = {'x1': 0.95, 'x2': 0.50, 'x3': 0.95}
    uut._handle_event('ASSIST_COMPLETE')
    assert uut.current_state == 'ASSISTING'
    uut.destroy_node()


def test_home_reached_goes_to_idle():
    uut = _make_fsm()
    uut.current_state = 'RETURNING'
    uut._handle_event('HOME_REACHED')
    assert uut.current_state == 'IDLE'
    uut.destroy_node()


# ===== Detection flow =====

def test_detection_event_from_searching_goes_to_verifying():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('DETECTION_EVENT')
    assert uut.current_state == 'VERIFYING'
    uut.destroy_node()


def test_detection_event_from_refining_goes_to_verifying():
    uut = _make_fsm()
    uut.current_state = 'REFINING'
    uut._handle_event('DETECTION_EVENT')
    assert uut.current_state == 'VERIFYING'
    uut.destroy_node()


def test_detection_event_from_assisting_goes_to_verifying():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('DETECTION_EVENT')
    assert uut.current_state == 'VERIFYING'
    uut.destroy_node()


def test_confirmed_target_goes_to_target_lock():
    uut = _make_fsm()
    uut.current_state = 'VERIFYING'
    uut._handle_event('CONFIRMED_TARGET')
    assert uut.current_state == 'TARGET_LOCK'
    uut.destroy_node()


def test_false_positive_resumes_prior_search_state():
    uut = _make_fsm()
    uut.current_state = 'VERIFYING'
    uut.previous_state = 'REFINING'
    uut._handle_event('FALSE_POSITIVE')
    assert uut.current_state == 'REFINING'
    uut.destroy_node()


def test_false_positive_falls_back_to_searching_if_prior_not_search():
    uut = _make_fsm()
    uut.current_state = 'VERIFYING'
    uut.previous_state = 'IDLE'
    uut._handle_event('FALSE_POSITIVE')
    assert uut.current_state == 'SEARCHING'
    uut.destroy_node()


def test_timeout_in_verifying_resumes_prior_state():
    uut = _make_fsm()
    uut.current_state = 'VERIFYING'
    uut.previous_state = 'ASSISTING'
    uut._handle_event('TIMEOUT')
    assert uut.current_state == 'ASSISTING'
    uut.destroy_node()


def test_target_lost_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'TARGET_LOCK'
    uut._handle_event('TARGET_LOST')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_command_return_from_target_lock_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'TARGET_LOCK'
    uut._handle_event('COMMAND_RETURN')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


# ===== Recovery flow =====

def test_path_failed_goes_to_recovery():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('PATH_FAILED')
    assert uut.current_state == 'RECOVERY'
    uut.destroy_node()


def test_collision_risk_goes_to_recovery():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('COLLISION_RISK')
    assert uut.current_state == 'RECOVERY'
    uut.destroy_node()


def test_comms_loss_goes_to_recovery():
    uut = _make_fsm()
    uut.current_state = 'REFINING'
    uut._handle_event('COMMS_LOSS')
    assert uut.current_state == 'RECOVERY'
    uut.destroy_node()


def test_replan_success_resumes_prior_state():
    uut = _make_fsm()
    uut.current_state = 'RECOVERY'
    uut.previous_state = 'REFINING'
    uut._handle_event('REPLAN_SUCCESS')
    assert uut.current_state == 'REFINING'
    uut.destroy_node()


def test_replan_fail_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'RECOVERY'
    uut._handle_event('REPLAN_FAIL')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


# ===== Safety priority overrides =====

def test_low_battery_overrides_searching():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('LOW_BATTERY')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_low_battery_overrides_assisting():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('LOW_BATTERY')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_low_battery_overrides_verifying():
    uut = _make_fsm()
    uut.current_state = 'VERIFYING'
    uut._handle_event('LOW_BATTERY')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_critical_failure_goes_to_emergency_stop():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('CRITICAL_FAILURE')
    assert uut.current_state == 'EMERGENCY_STOP'
    uut.destroy_node()


def test_hard_collision_goes_to_emergency_stop():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('HARD_COLLISION')
    assert uut.current_state == 'EMERGENCY_STOP'
    uut.destroy_node()


def test_emergency_stop_ignores_all_events_except_reset():
    uut = _make_fsm()
    uut.current_state = 'EMERGENCY_STOP'
    for event in ('MISSION_START', 'LOW_BATTERY', 'MISSION_STOP', 'PATH_FAILED'):
        uut._handle_event(event)
        assert uut.current_state == 'EMERGENCY_STOP', f'{event} should not exit EMERGENCY_STOP'
    uut.destroy_node()


def test_reset_exits_emergency_stop_to_idle():
    uut = _make_fsm()
    uut.current_state = 'EMERGENCY_STOP'
    uut._handle_event('RESET')
    assert uut.current_state == 'IDLE'
    uut.destroy_node()


def test_mission_stop_goes_to_returning():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('MISSION_STOP')
    assert uut.current_state == 'RETURNING'
    uut.destroy_node()


def test_mission_update_restarts_searching():
    uut = _make_fsm()
    uut.current_state = 'ASSISTING'
    uut._handle_event('MISSION_UPDATE')
    assert uut.current_state == 'SEARCHING'
    uut.destroy_node()


# ===== Previous state tracking =====

def test_previous_state_updated_on_transition():
    uut = _make_fsm()
    uut.current_state = 'SEARCHING'
    uut._handle_event('DETECTION_EVENT')
    assert uut.previous_state == 'SEARCHING'
    uut.destroy_node()


# ===== Pub/sub integration =====

def test_state_published_on_mission_start():
    uut = UAVStateManager()
    helper = rclpy.create_node('test_fsm_start_helper')
    start_pub = helper.create_publisher(Empty, '/mission/start', 10)
    received = []
    helper.create_subscription(UAVState, '/uav/state', received.append, 10)

    _spin(uut, helper, 10)
    start_pub.publish(Empty())
    _spin(uut, helper, 20)

    states = [m.state for m in received]
    assert 'SEARCHING' in states

    uut.destroy_node()
    helper.destroy_node()


def test_fsm_event_triggers_transition():
    uut = UAVStateManager()
    helper = rclpy.create_node('test_fsm_event_helper')
    event_pub = helper.create_publisher(FSMEvent, '/x1/fsm/event', 10)
    received = []
    helper.create_subscription(UAVState, '/uav/state', received.append, 10)

    uut.current_state = 'SEARCHING'

    _spin(uut, helper, 10)

    msg = FSMEvent()
    msg.uav_id = 'x1'
    msg.event = 'REGION_COMPLETE'
    msg.value = 0.7
    msg.timestamp = 0.0
    event_pub.publish(msg)
    _spin(uut, helper, 20)

    states = [m.state for m in received]
    assert 'REFINING' in states

    uut.destroy_node()
    helper.destroy_node()


def test_target_detected_published_on_target_lock():
    uut = UAVStateManager()
    helper = rclpy.create_node('test_fsm_targetlock_helper')
    detected = []
    helper.create_subscription(String, '/gcs/target_detected', detected.append, 10)

    uut.current_state = 'VERIFYING'

    _spin(uut, helper, 10)
    uut._handle_event('CONFIRMED_TARGET')
    _spin(uut, helper, 20)

    assert len(detected) == 1
    assert 'x1' in detected[0].data

    uut.destroy_node()
    helper.destroy_node()


# ===== SwarmCoordinator =====

def _make_state_msg(uav_id, state, previous_state='IDLE'):
    msg = UAVState()
    msg.uav_id = uav_id
    msg.state = state
    msg.previous_state = previous_state
    msg.timestamp = 0.0
    return msg


def _make_coordinator(uav_id='x1', num_uavs=3, rows=3, threshold=0.95):
    return SwarmCoordinator()


# ===== Lawnmower helper =====

def test_lawnmower_returns_2_poses_per_row():
    poses = _lawnmower(0.0, 10.0, 0.0, 10.0, rows=4)
    assert len(poses) == 8   # 4 rows × 2 endpoints


def test_lawnmower_alternates_direction():
    poses = _lawnmower(0.0, 10.0, 0.0, 10.0, rows=2)
    # row 0: left→right (x_start first)
    assert poses[0].position.x == pytest.approx(0.0)
    assert poses[1].position.x == pytest.approx(10.0)
    # row 1: right→left (x_end first)
    assert poses[2].position.x == pytest.approx(10.0)
    assert poses[3].position.x == pytest.approx(0.0)


def test_lawnmower_single_row_stays_at_ymin():
    poses = _lawnmower(0.0, 5.0, -3.0, 3.0, rows=1)
    assert all(p.position.y == pytest.approx(-3.0) for p in poses)


# ===== Region assignment =====

def test_region_sliced_correctly_for_x1():
    uut = SwarmCoordinator()
    # default: area=[-10,10], num_uavs=3, uav_id='x1' (index 0)
    # slice_width = 20/3 ≈ 6.667
    assert uut.x_start == pytest.approx(-10.0)
    assert uut.x_end == pytest.approx(-10.0 + 20.0 / 3)
    uut.destroy_node()


# ===== Waypoints published on SEARCHING =====

def test_waypoints_published_on_searching_state():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_search_helper')
    received = []

    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    helper.create_subscription(PoseArray, '/x1/nav/waypoints', received.append, qos)

    _spin(uut, helper, 10)
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))
    _spin(uut, helper, 20)

    assert len(received) == 1
    assert len(received[0].poses) == 3 * 2   # rows=3, 2 endpoints each

    uut.destroy_node()
    helper.destroy_node()


def test_waypoints_not_published_for_other_uav():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_other_uav_helper')
    received = []

    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    helper.create_subscription(PoseArray, '/x1/nav/waypoints', received.append, qos)

    _spin(uut, helper, 10)
    uut._on_fsm_state_change(_make_state_msg('x2', 'SEARCHING', 'IDLE'))
    _spin(uut, helper, 20)

    assert len(received) == 0

    uut.destroy_node()
    helper.destroy_node()


def test_refinement_waypoints_are_denser():
    uut = _make_coordinator()

    search_count = 0
    refine_count = 0

    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))
    search_count = uut.coverage_waypoints_total

    uut._on_fsm_state_change(_make_state_msg('x1', 'REFINING', 'SEARCHING'))
    refine_count = uut.coverage_waypoints_total

    assert refine_count == search_count * 2

    uut.destroy_node()


# ===== Coverage tracking =====

def test_waypoint_reached_increments_visited():
    uut = _make_coordinator()
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))

    uut._on_waypoint_reached(Empty())
    uut._on_waypoint_reached(Empty())

    assert uut.coverage_waypoints_visited == 2
    uut.destroy_node()


def test_waypoint_reached_ignored_when_paused():
    uut = _make_coordinator()
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))
    uut._on_fsm_state_change(_make_state_msg('x1', 'VERIFYING', 'SEARCHING'))

    uut._on_waypoint_reached(Empty())
    uut._on_waypoint_reached(Empty())

    assert uut.coverage_waypoints_visited == 0
    uut.destroy_node()


def test_coverage_not_reset_when_resuming_from_verifying():
    uut = _make_coordinator()
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))

    uut._on_waypoint_reached(Empty())
    assert uut.coverage_waypoints_visited == 1

    # enter VERIFYING — pauses counting
    uut._on_fsm_state_change(_make_state_msg('x1', 'VERIFYING', 'SEARCHING'))
    # resume — should NOT reset
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'VERIFYING'))

    assert uut.coverage_waypoints_visited == 1

    uut.destroy_node()


def test_coverage_reset_on_fresh_mode_entry():
    uut = _make_coordinator()
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))
    uut._on_waypoint_reached(Empty())

    # transition to REFINING (fresh, not from VERIFYING)
    uut._on_fsm_state_change(_make_state_msg('x1', 'REFINING', 'SEARCHING'))

    assert uut.coverage_waypoints_visited == 0
    uut.destroy_node()


# ===== Coverage events =====

def test_region_complete_published_when_all_waypoints_visited():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_region_complete_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))

    total = uut.coverage_waypoints_total
    for _ in range(total):
        uut._on_waypoint_reached(Empty())

    _spin(uut, helper, 20)

    assert any(e.event == 'REGION_COMPLETE' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_region_complete_value_is_coverage_ratio():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_coverage_value_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)
    uut._on_fsm_state_change(_make_state_msg('x1', 'SEARCHING', 'IDLE'))

    total = uut.coverage_waypoints_total
    for _ in range(total):
        uut._on_waypoint_reached(Empty())

    _spin(uut, helper, 20)

    region_events = [e for e in events if e.event == 'REGION_COMPLETE']
    assert len(region_events) == 1
    assert region_events[0].value == pytest.approx(1.0)

    uut.destroy_node()
    helper.destroy_node()


def test_refinement_complete_published_after_refining():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_refine_complete_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)
    uut._on_fsm_state_change(_make_state_msg('x1', 'REFINING', 'SEARCHING'))

    total = uut.coverage_waypoints_total
    for _ in range(total):
        uut._on_waypoint_reached(Empty())

    _spin(uut, helper, 20)

    assert any(e.event == 'REFINEMENT_COMPLETE' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_all_drones_done_when_all_coverage_complete():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_all_done_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)

    # seed coverage_map: all UAVs at full coverage
    uut.coverage_map = {'x1': 1.0, 'x2': 1.0, 'x3': 1.0}
    uut._on_fsm_state_change(_make_state_msg('x1', 'ASSISTING', 'REFINING'))

    _spin(uut, helper, 20)

    # should immediately emit ALL_DRONES_DONE since all coverage >= threshold
    assert any(e.event == 'ALL_DRONES_DONE' for e in events)

    uut.destroy_node()
    helper.destroy_node()


def test_assist_complete_when_other_regions_unfinished():
    uut = _make_coordinator()
    helper = rclpy.create_node('test_coord_assist_complete_helper')
    events = []
    helper.create_subscription(FSMEvent, '/x1/fsm/event', events.append, 10)

    _spin(uut, helper, 10)

    # x2 still below threshold
    uut.coverage_map = {'x1': 1.0, 'x2': 0.5, 'x3': 1.0}
    uut._on_fsm_state_change(_make_state_msg('x1', 'ASSISTING', 'REFINING'))

    total = uut.coverage_waypoints_total
    for _ in range(total):
        uut._on_waypoint_reached(Empty())

    _spin(uut, helper, 20)

    assert any(e.event == 'ASSIST_COMPLETE' for e in events)

    uut.destroy_node()
    helper.destroy_node()


# ===== Coverage map update =====

def test_coverage_map_updated_from_mission_coverage():
    uut = _make_coordinator()

    msg = MissionCoverage()
    msg.uav_ids = ['x1', 'x2', 'x3']
    msg.coverage_ratios = [0.5, 0.8, 0.3]
    msg.all_complete = False
    msg.timestamp = 0.0

    uut._on_coverage_update(msg)

    assert uut.coverage_map['x1'] == pytest.approx(0.5)
    assert uut.coverage_map['x2'] == pytest.approx(0.8)
    assert uut.coverage_map['x3'] == pytest.approx(0.3)

    uut.destroy_node()
