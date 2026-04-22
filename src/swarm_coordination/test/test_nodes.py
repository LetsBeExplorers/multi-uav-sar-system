# Copyright 2026 raytracer
# Licensed under the MIT License.
"""Functional tests for swarm coordination nodes."""

import pytest
import rclpy
from sar_msgs.msg import FSMEvent, UAVState
from std_msgs.msg import Empty, String
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
