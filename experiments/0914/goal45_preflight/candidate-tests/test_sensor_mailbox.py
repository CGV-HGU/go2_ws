"""Concurrency/fault regressions for the isolated sensor-reception candidate."""
import math
import threading
import time
from types import SimpleNamespace as NS

import pytest
from nav_msgs.msg import Odometry
from s2e_vlm_msgs.msg import StampedPose
from s2e_vlm_robot.motion import MotionLimits, MotionSample, MeasuredMotion
from test_robot_boundary import node, enable, command


def pose(n):
    p = StampedPose()
    p.header.stamp = n.get_clock().now().to_msg()
    p.header.frame_id = 'robot_origin'
    p.pose.orientation.w = 1.
    p.status = 'OK'
    return p


def test_pose_reception_completes_while_control_lock_is_held(node):
    n, _ = node
    held, release, received = threading.Event(), threading.Event(), threading.Event()
    def hold_control():
        with n._lock:
            held.set()
            release.wait(2.)
    thread = threading.Thread(target=hold_control)
    thread.start()
    assert held.wait(1.)
    p = pose(n)
    worker = threading.Thread(target=lambda: (n._on_pose(p), received.set()))
    worker.start()
    try:
        assert received.wait(.3), 'Sensor callback waited for the held control lock'
        assert n._last_pose.header == p.header
    finally:
        release.set()
        thread.join(2.)
        worker.join(2.)


def test_sensor_fault_is_latched_and_stops_before_next_move(node):
    n, transport = node
    enable(n)
    n._on_command(command(n))
    invalid = Odometry()
    invalid.header.stamp = n.get_clock().now().to_msg()
    invalid.header.frame_id = 'odom'
    invalid.child_frame_id = 'base_link'
    invalid.pose.pose.orientation.w = 0.
    n._on_odom(invalid)  # Zero quaternion cannot be a usable odometry sample.
    assert n._pending_sensor_fault == 'INVALID_ODOMETRY'
    assert n._admission_error(time.monotonic()) == 'INVALID_ODOMETRY'
    n._tick()
    assert not n._motion_enabled and n._active is None
    assert n._last_fault == 'INVALID_ODOMETRY'
    transport.move.assert_not_called()
    assert transport.stop.called


def test_disable_waits_for_inflight_move_then_prevents_any_later_move(node):
    n, transport = node
    enable(n)
    n._on_command(command(n))
    entered, release, disabled = threading.Event(), threading.Event(), threading.Event()
    events = []
    def move(*_):
        events.append('move_begin')
        entered.set()
        assert release.wait(2.)
        events.append('move_end')
    def disable():
        result = n._set_motion_enabled(NS(data=False), NS())
        assert result.success
        events.append('disable_ack')
        disabled.set()
    transport.move.side_effect = move
    transport.stop.side_effect = lambda: events.append('stop')
    control = threading.Thread(target=n._tick)
    control.start()
    stopper = None
    try:
        assert entered.wait(1.)
        n._on_pose(pose(n))  # Reception remains possible during blocked Move.
        stopper = threading.Thread(target=disable)
        stopper.start()
        release.set()
        assert disabled.wait(1.)
        control.join(1.)
        stopper.join(1.)
        count = transport.move.call_count
        n._tick()
        assert transport.move.call_count == count == 1
        assert events.index('move_end') < events.index('disable_ack')
        assert not n._motion_enabled
    finally:
        release.set()
        control.join(2.)
        if stopper is not None:
            stopper.join(2.)


@pytest.mark.parametrize('spread', [0., -1., float('nan'), math.radians(1.01)])
def test_invalid_rest_spread_is_rejected(spread):
    with pytest.raises(ValueError):
        MotionLimits(rotation_settle_spread=spread)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_one_degree_window_accepts_bounded_noise_but_not_continuing_drift(sign):
    def sample(t, angle, rate=0.):
        return MotionSample(1+round(t*1e9), t, 0., 0., math.radians(sign*angle), 0., sign*rate)
    limits = MotionLimits(rotation_settle_window=.35, rotation_settle_spread=math.radians(1.))
    drift = MeasuredMotion(kind='rotate', target=sign*math.radians(30), sample=sample(0, 0), now=0., timeout=15., limits=limits)
    drift.step(sample(1., 29., .2), now=1.)
    # Recorded recoil regression: 0.18 degrees per 50ms is not rest.
    for i in range(1, 16):
        t=1.+i*.05
        assert not drift.step(sample(t, 29.-i*.18, -.06), now=t).terminal
    noise = MeasuredMotion(kind='rotate', target=sign*math.radians(30), sample=sample(0, 0), now=0., timeout=15., limits=limits)
    noise.step(sample(1., 29., .2), now=1.)
    for i in range(1, 12):
        t=1.+i*.05
        out=noise.step(sample(t, 29.4 + (.2 if i%2 else -.2), .08), now=t)
        if out.terminal:
            break
    assert out.terminal == 'OK'
    assert abs(out.progress-noise.target) <= noise.tolerance
