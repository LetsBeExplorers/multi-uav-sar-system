"""Functional tests for detection and verification nodes."""

import pytest
import rclpy
from human_detection.detection_node import DetectionNode
from human_detection.verification_node import VerificationNode
from sar_msgs.msg import FSMEvent


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# ===== Detection node =====

def test_detection_below_threshold_does_not_publish():
    uut = DetectionNode()
    uut.confidence_threshold = 0.5
    uut.persistence_threshold = 1
    published = []
    uut._fsm_pub.publish = lambda msg: published.append(msg)
    # simulate a low-confidence "look" by patching the random draw
    import human_detection.detection_node as mod
    orig = mod.random.random
    mod.random.random = lambda: 0.0  # always trigger fake detection
    mod.random.uniform = lambda a, b: 0.3  # below threshold
    try:
        uut._tick()
    finally:
        mod.random.random = orig
    assert published == []
    assert uut.consecutive_detections == 0
    uut.destroy_node()


def test_detection_requires_persistence():
    uut = DetectionNode()
    uut.confidence_threshold = 0.5
    uut.persistence_threshold = 3
    uut.fake_prob = 1.0
    published = []
    uut._fsm_pub.publish = lambda msg: published.append(msg)
    uut._detection_pub.publish = lambda msg: None

    import human_detection.detection_node as mod
    mod.random.random = lambda: 0.0
    mod.random.uniform = lambda a, b: 0.9  # always passes threshold

    uut._tick()
    uut._tick()
    assert published == []  # not enough yet
    uut._tick()
    assert len(published) == 1
    assert published[0].event == 'DETECTION_EVENT'
    uut.destroy_node()


# ===== Verification node =====

def test_verification_ignores_other_uavs_commands():
    uut = VerificationNode()
    msg = FSMEvent()
    msg.uav_id = 'x2'  # not us
    msg.event = 'START_VERIFY'
    uut._on_command(msg)
    assert uut._verify_timer is None
    uut.destroy_node()


def test_verification_starts_timer_on_start_verify():
    uut = VerificationNode()
    msg = FSMEvent()
    msg.uav_id = uut.uav_id
    msg.event = 'START_VERIFY'
    uut._on_command(msg)
    assert uut._verify_timer is not None
    uut._verify_timer.cancel()
    uut.destroy_node()


def test_verification_decide_publishes_confirmed_or_false():
    uut = VerificationNode()
    published = []
    uut._fsm_pub.publish = lambda msg: published.append(msg)

    import human_detection.verification_node as mod
    mod.random.random = lambda: 0.0  # below confirm_prob → CONFIRMED
    uut._decide()
    assert published[-1].event == 'CONFIRMED_TARGET'

    mod.random.random = lambda: 1.0  # above → FALSE_POSITIVE
    uut._decide()
    assert published[-1].event == 'FALSE_POSITIVE'

    uut.destroy_node()