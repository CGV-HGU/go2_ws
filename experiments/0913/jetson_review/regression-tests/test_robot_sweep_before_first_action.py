"""Reproduce the 20260913 pause accepted before a policy's first action.

Use real PixNav/sweep callbacks with typed messages; no ROS graph or actuator.
"""
from types import SimpleNamespace as NS

import pytest
from builtin_interfaces.msg import Time
from s2e_vlm_msgs.msg import SweepAck
from s2e_vlm_core.sweep_prefetch import SWEEP_CHECKPOINT_REASON
from s2e_vlm_nodes.runtime.pixnav import PixNavRuntimeNode
from .test_robot_async_correction_handoff import (
    command_message, local_request, pixel_node, request_message, scheduler,
)
from .test_robot_physical_body_look import runtime
from .test_pixnav_runtime_node import frame


def test_zero_action_checkpoint_completes_vlm_owner_without_navigation_failure(monkeypatch):
    from dataclasses import replace
    from unittest.mock import patch
    from s2e_vlm_nodes.runtime.vlm import VlmMockNode
    from .test_robot_terminal_feedback_lifetime import waiting_node
    from .test_robot_sweep_checkpoint_feedback import feedback_node
    node, old = waiting_node(monkeypatch)
    result = replace(old, reason=SWEEP_CHECKPOINT_REASON,
                     action_count=0, step_index=0, policy_actions=())
    pose_node, _ = feedback_node(0.)
    pose = pose_node._latest_pose_message()
    pose.header.stamp.sec = 1
    pose.header.stamp.nanosec = 0
    node._latest_pose_message = lambda: pose
    with patch.object(VlmMockNode, '_record_pixnav_navigation_outcome') as outcome:
        VlmMockNode._process_pixnav_policy_result(node, result)
    assert node.sync.gate.outcome == 'PIXNAV_LOCAL_POLICY_COMPLETE'
    assert node.reference_pending_go_feedback is None
    node._publish_pixnav_feedback_ack.assert_called_once_with('active')
    node._queue_async_blocked_go_observation_recovery.assert_not_called()
    outcome.assert_not_called()


@pytest.mark.parametrize('action', ['move_forward', 'look_down', 'look_up', 'turn_left', 'stop'])
@pytest.mark.parametrize('image_order', ['cached', 'next', 'stale_then_next'])
def test_pause_before_first_action_emits_checkpoint_without_executing_policy(
        monkeypatch, action, image_order):
    monkeypatch.setenv('PIXNAV_LOOK_EXECUTION', 'forward_0p1')
    r = runtime((action, 'stop'))
    assert r._last_result is None and r._action_count == 0
    s = scheduler()
    s._on_local_policy_request(local_request(request_id=r.active_request.request_id,
                                            generation=r._active_generation))
    request_message(monkeypatch, s)
    pause = command_message(s._publish_command.call_args.args[0])
    n, outputs = pixel_node(r)
    results, acks = [], []
    n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1300, to_msg=lambda: Time(nanosec=1300)))
    n.result_publisher = NS(publish=results.append)
    n.sweep_ack_publisher = NS(publish=acks.append)
    n._publish_sweep_paused_ack = lambda result: PixNavRuntimeNode._publish_sweep_paused_ack(n, result)
    def publish(output):
        outputs.append(output)
        PixNavRuntimeNode._publish_step_output(n, output)
    n._publish_step_output = publish
    if image_order == 'cached':
        r.append_image(frame(stamp_ns=1200))
    PixNavRuntimeNode._on_sweep_command(n, pause)
    if image_order != 'cached':
        assert not outputs
        if image_order == 'stale_then_next':
            assert not r.can_step_with_image(frame(stamp_ns=100))
        publish(r.step(frame(stamp_ns=1200), now_ns=1300))
    assert len(outputs) == len(results) == len(acks) == 1
    out = outputs[0]
    assert out.command is None
    assert out.result.status == 'OK'
    assert out.result.reason == SWEEP_CHECKPOINT_REASON
    assert out.result.action_count == 0 and out.result.policy_actions == ()
    assert out.result.decision_id == pause.source_decision_id
    assert out.result.result_stamp_ns >= 1200
    assert not r.worker.step_calls
    assert r.active_request is None and not r.action_in_flight
    assert acks[0].ack == SweepAck.EXECUTOR_PAUSED
    assert acks[0].sweep_id == pause.sweep_id
    assert acks[0].source_decision_id == pause.source_decision_id
