"""Functional tests for detection and verification nodes."""

import pytest
import rclpy
from human_detection.detection_node import DetectionNode
from human_detection.verification_node import VerificationNode
from sar_msgs.msg import FSMEvent, DetectionEvent
import human_detection.detection_node as detection_mod
import human_detection.verification_node as verification_mod


# ===== Fixtures =====

@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def detection_node():
    """A DetectionNode with all publishers stubbed out and sensible defaults."""
    node = DetectionNode()
    node.confidence_threshold = 0.5
    node.persistence_threshold = 3
    node.fake_prob = 1.0  # always trigger fake detection unless overridden
    node._fsm_pub.publish = lambda msg: node._fsm_published.append(msg)
    node._detection_pub.publish = lambda msg: node._detection_published.append(msg)
    node._alert_pub.publish = lambda msg: node._alert_published.append(msg)
    node._fsm_published = []
    node._detection_published = []
    node._alert_published = []
    # Pre-arm mission so _tick runs without needing /mission/start.
    node.mission_active = True
    node.start_time = node.get_clock().now().nanoseconds / 1e9 - 100.0  # past warmup
    yield node
    node.destroy_node()


@pytest.fixture
def verification_node():
    """A VerificationNode with publishers stubbed out."""
    node = VerificationNode()
    node._fsm_pub.publish = lambda msg: node._fsm_published.append(msg)
    node._alert_pub.publish = lambda msg: node._alert_published.append(msg)
    node._fsm_published = []
    node._alert_published = []
    yield node
    node.destroy_node()


# ===== Detection node =====

def test_detection_below_threshold_does_not_publish(detection_node, monkeypatch):
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)  # fake fires
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.3)  # < 0.5

    detection_node._tick()

    assert detection_node._fsm_published == []
    assert detection_node.consecutive_detections == 0


def test_detection_requires_persistence(detection_node, monkeypatch):
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)

    detection_node._tick()
    detection_node._tick()
    assert detection_node._fsm_published == []  # not enough yet

    detection_node._tick()
    assert len(detection_node._fsm_published) == 1
    assert detection_node._fsm_published[0].event == 'DETECTION_EVENT'
    # Confidence is now exposed on the FSM event.
    assert detection_node._fsm_published[0].value == pytest.approx(0.9)


def test_detection_suppressed_during_warmup(detection_node, monkeypatch):
    # Reset start_time to "now" so we're inside the warmup window.
    detection_node.start_time = detection_node.get_clock().now().nanoseconds / 1e9
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)

    for _ in range(10):
        detection_node._tick()

    assert detection_node._fsm_published == []
    assert detection_node.consecutive_detections == 0


def test_detection_suppressed_when_mission_inactive(detection_node, monkeypatch):
    detection_node.mission_active = False
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)

    for _ in range(10):
        detection_node._tick()

    assert detection_node._fsm_published == []


def test_on_stop_clears_state(detection_node):
    detection_node.consecutive_detections = 2
    detection_node.detection_active = True

    detection_node._on_stop(None)

    assert detection_node.mission_active is False
    assert detection_node.consecutive_detections == 0
    assert detection_node.detection_active is False


def test_detection_re_triggers_after_confidence_drop(detection_node, monkeypatch):
    """After a streak fires, a confidence drop should re-arm the edge trigger."""
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)

    # First streak — fire once.
    for _ in range(detection_node.persistence_threshold):
        detection_node._tick()
    assert len(detection_node._fsm_published) == 1

    # Drop below threshold — should reset detection_active.
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.1)
    detection_node._tick()
    assert detection_node.detection_active is False
    assert detection_node.consecutive_detections == 0

    # New streak — should fire again.
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)
    for _ in range(detection_node.persistence_threshold):
        detection_node._tick()
    assert len(detection_node._fsm_published) == 2


def test_detection_alert_uses_documented_type(detection_node, monkeypatch):
    monkeypatch.setattr(detection_mod.random, 'random', lambda: 0.0)
    monkeypatch.setattr(detection_mod.random, 'uniform', lambda a, b: 0.9)

    for _ in range(detection_node.persistence_threshold):
        detection_node._tick()

    assert len(detection_node._alert_published) == 1
    # Alert.msg documents type as one of: DETECTION, CONFIRMATION, ERROR.
    assert detection_node._alert_published[0].type == 'DETECTION'
    assert detection_node._alert_published[0].level == 'WARNING'


# ===== Verification node =====

def test_verification_ignores_other_uavs_commands(verification_node):
    msg = FSMEvent()
    msg.uav_id = 'x2'  # not us
    msg.event = 'START_VERIFY'
    verification_node._on_command(msg)
    assert verification_node._verify_timer is None


def test_verification_ignores_unknown_events(verification_node):
    msg = FSMEvent()
    msg.uav_id = verification_node.uav_id
    msg.event = 'SOMETHING_ELSE'
    verification_node._on_command(msg)
    assert verification_node._verify_timer is None


def test_verification_starts_timer_on_start_verify(verification_node):
    msg = FSMEvent()
    msg.uav_id = verification_node.uav_id
    msg.event = 'START_VERIFY'
    verification_node._on_command(msg)
    assert verification_node._verify_timer is not None
    assert verification_node._timeout_timer is not None


def test_verification_second_start_replaces_first_timer(verification_node):
    msg = FSMEvent()
    msg.uav_id = verification_node.uav_id
    msg.event = 'START_VERIFY'

    verification_node._on_command(msg)
    first = verification_node._verify_timer

    verification_node._on_command(msg)
    second = verification_node._verify_timer

    assert first is not second
    assert verification_node._verify_timer is not None


def test_verification_decide_publishes_confirmed(verification_node, monkeypatch):
    monkeypatch.setattr(verification_mod.random, 'random', lambda: 0.0)  # < confirm_prob
    verification_node._decide()
    assert verification_node._fsm_published[-1].event == 'CONFIRMED_TARGET'
    assert verification_node._alert_published[-1].level == 'CRITICAL'
    assert verification_node._alert_published[-1].type == 'CONFIRMATION'


def test_verification_decide_publishes_false_positive(verification_node, monkeypatch):
    monkeypatch.setattr(verification_mod.random, 'random', lambda: 1.0)  # > confirm_prob
    verification_node._decide()
    assert verification_node._fsm_published[-1].event == 'FALSE_POSITIVE'
    assert verification_node._alert_published[-1].level == 'INFO'


def test_confirmed_alert_includes_cached_location(verification_node, monkeypatch):
    """On CONFIRMED_TARGET, the alert message should embed the cached x/y/conf."""
    detection = DetectionEvent()
    detection.uav_id = verification_node.uav_id
    detection.x = 5.2
    detection.y = -3.4
    detection.confidence = 0.87
    detection.timestamp = 0.0
    verification_node._on_detection(detection)

    monkeypatch.setattr(verification_mod.random, 'random', lambda: 0.0)
    verification_node._decide()

    msg = verification_node._alert_published[-1].message
    assert '5.2' in msg
    assert '-3.4' in msg
    assert '0.87' in msg


def test_confirmed_alert_without_detection_falls_back(verification_node, monkeypatch):
    """If no detection was ever cached, the alert should not crash and should signal unknown."""
    monkeypatch.setattr(verification_mod.random, 'random', lambda: 0.0)
    verification_node._decide()

    msg = verification_node._alert_published[-1].message
    assert 'unknown' in msg.lower()


def test_detection_for_other_uav_is_ignored(verification_node, monkeypatch):
    """A DetectionEvent from another UAV must not pollute our cached location."""
    other = DetectionEvent()
    other.uav_id = 'x99'
    other.x = 99.0
    other.y = 99.0
    other.confidence = 0.99
    other.timestamp = 0.0
    verification_node._on_detection(other)

    assert verification_node._last_detection is None


def test_verification_timeout_publishes_timeout_event(verification_node):
    verification_node._on_timeout()
    assert verification_node._fsm_published[-1].event == 'TIMEOUT'
    assert verification_node._alert_published[-1].type == 'ERROR'


def test_verification_decide_clears_timers(verification_node, monkeypatch):
    msg = FSMEvent()
    msg.uav_id = verification_node.uav_id
    msg.event = 'START_VERIFY'
    verification_node._on_command(msg)
    assert verification_node._verify_timer is not None

    monkeypatch.setattr(verification_mod.random, 'random', lambda: 0.0)
    verification_node._decide()

    assert verification_node._verify_timer is None
    assert verification_node._timeout_timer is None