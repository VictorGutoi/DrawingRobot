"""Unit tests for the publish-throttle decision.

Only tests the pure static method `_should_publish` — instantiating
`RosPublisher` requires rclpy, which is not assumed to be installed in CI.
"""

from drawingrobot.ros_publisher import RosPublisher


HEARTBEAT_S = 1.0


def test_nonzero_always_publishes_regardless_of_recency():
    # Just published 1ms ago, but next sample is non-zero → must go out.
    assert RosPublisher._should_publish(
        is_zero=False, force=False, now=10.001,
        last_pub_t=10.000, heartbeat_s=HEARTBEAT_S)


def test_force_always_publishes_even_when_idle_recent():
    # Force the final-stop Twist at shutdown, regardless of heartbeat timing.
    assert RosPublisher._should_publish(
        is_zero=True, force=True, now=10.001,
        last_pub_t=10.000, heartbeat_s=HEARTBEAT_S)


def test_zero_within_heartbeat_window_is_suppressed():
    assert not RosPublisher._should_publish(
        is_zero=True, force=False, now=10.5,
        last_pub_t=10.0, heartbeat_s=HEARTBEAT_S)


def test_zero_after_heartbeat_window_publishes():
    assert RosPublisher._should_publish(
        is_zero=True, force=False, now=11.5,
        last_pub_t=10.0, heartbeat_s=HEARTBEAT_S)


def test_first_call_publishes_even_if_zero():
    # last_pub_t = -inf at init, so the first zero is honored as the leading
    # "system alive" sample.
    assert RosPublisher._should_publish(
        is_zero=True, force=False, now=0.0,
        last_pub_t=float("-inf"), heartbeat_s=HEARTBEAT_S)


def test_exact_heartbeat_boundary_publishes():
    # >= boundary, not > — so exactly heartbeat_s elapsed is enough.
    assert RosPublisher._should_publish(
        is_zero=True, force=False, now=11.0,
        last_pub_t=10.0, heartbeat_s=HEARTBEAT_S)
