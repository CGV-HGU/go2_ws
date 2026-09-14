"""Exercise ROS boundary callbacks with a recording transport, never an actuator."""
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from s2e_vlm_msgs.msg import HabitatActionCommand, NavigationTask, StampedPose, SweepCommand, SweepAck, NodeStatus, PointGoal
from s2e_vlm_robot import action_executor_node as module
from s2e_vlm_robot.motion import MotionSample, MotionOutput
from s2e_vlm_robot.rtab_pointgoal_node import RtabPointGoalNode
from sensor_msgs.msg import CameraInfo, Image


@pytest.fixture
def node(monkeypatch):
    transport = Mock()
    monkeypatch.setattr(module, "SportClient", lambda *args: transport)
    rclpy.init()
    node = module.ActionExecutorNode()
    node._result_publisher = Mock()
    node._ack_publisher = Mock()
    now = time.monotonic()
    stamp = node.get_clock().now().nanoseconds
    node._odom = MotionSample(stamp, now, 0, 0, 0, 0, 0)
    node._last_pose = StampedPose()
    node._last_pose.header.stamp = node.get_clock().now().to_msg()
    node._last_pose.header.frame_id = "robot_origin"
    node._last_pose.pose.orientation.w = 1.
    node._last_pose.status = "OK"
    node._camera_received = now
    node._health = {n: (now, True) for n in ("vlm_node", "pixnav_runtime_node", "sweep_scheduler_node", "rtab_pointgoal_adapter")}
    node._episode_id = "episode"
    yield node, transport
    node.destroy_node()
    rclpy.shutdown()


def command(node, name="move_forward", identity="cmd"):
    c = HabitatActionCommand()
    c.header.stamp = node.get_clock().now().to_msg()
    c.episode_id = "episode"
    c.command_id = identity
    c.trajectory_id = "trajectory"
    c.generation = 1
    c.action = name
    c.forward_distance_m = .25
    c.turn_angle_deg = 30.
    return c


def enable(node):
    response = node._set_motion_enabled(NS(data=True), NS())
    assert response.success, response.message
    node._pointgoal = PointGoal()
    node._pointgoal.episode_id = "episode"
    node._pointgoal.header.stamp = node.get_clock().now().to_msg()


def finish_at_yaw(n, yaw):
    """Measured callback boundary fixture; does not simulate robot motion."""
    from dataclasses import replace
    stamp = n.get_clock().now().nanoseconds
    n._odom = replace(n._odom, stamp_ns=stamp, received_s=time.monotonic(), yaw=yaw)
    n._last_pose.header.stamp = n.get_clock().now().to_msg()
    n._finish(MotionOutput(terminal="OK"))
    assert n._pending_result is None


def test_forward_heading_survives_vlm_handoff_and_stationary_stop(node):
    n, transport = node
    enable(n)
    n._on_command(command(n))
    finish_at_yaw(n, math.radians(2))
    n._on_command(command(n, name="stop", identity="policy-stop"))
    next_command = command(n, identity="next-generation")
    next_command.generation = 2
    next_command.trajectory_id = "next-vlm-goal"
    n._on_command(next_command)
    assert n._active.start.yaw == pytest.approx(math.radians(2))
    assert n._active.heading_yaw == pytest.approx(0.)
    n._tick()
    assert transport.move.call_args.args[1] < 0


def hold_body_look(n, transport):
    from rclpy.parameter import Parameter
    from s2e_vlm_robot.sport_client import BodyStatus
    n.set_parameters([Parameter('body_look_enabled', value=True)])
    enable(n)
    look = command(n, name='look_down', identity='physical-look')
    n._on_command(look)
    n._on_command(look)  # Duplicate delivery must neither restart nor fail it.
    transport.body_look.assert_called_once_with('look_down')
    transport.body_status.return_value = BodyStatus(1, 99, 0, 3., '')
    n._tick()
    n._result_publisher.publish.assert_not_called()
    transport.body_status.return_value = BodyStatus(2, 99, n._odom.stamp_ns, 4.3, '')
    n._tick()
    assert n._result_publisher.publish.call_args.args[0].executed
    assert n._body_held
    transport.move.assert_not_called()


def test_physical_look_holds_policy_view_then_waits_for_return_before_move(node):
    from s2e_vlm_robot.sport_client import BodyStatus
    n, transport = node
    hold_body_look(n, transport)
    advance = command(n, identity='move-after-look')
    n._on_command(advance)
    n._on_command(advance)
    transport.body_return.assert_called_once()
    transport.body_status.return_value = BodyStatus(3, 99, 0, 2., '')
    n._tick()
    transport.move.assert_not_called()
    transport.body_status.return_value = BodyStatus(4, 99, n._odom.stamp_ns, 0., '')
    n._tick()
    assert n._active.kind == 'translate'
    assert not n._body_held and not n._body_returning
    assert transport.move.called


def test_planner_failure_returns_neutral_then_latches_without_queued_motion(node):
    from s2e_vlm_robot.sport_client import BodyStatus
    n,t=node;hold_body_look(n,t)
    n._on_command(command(n,identity='queued-before-fault'))
    t.stop.reset_mock()
    n._health['vlm_node']=(time.monotonic(),False)
    t.body_status.return_value=BodyStatus(3,99,0,14.6,'')
    n._tick()
    assert n._body_recovery_reason=='NODE_UNAVAILABLE:vlm_node'
    assert n._motion_enabled and n._active is None
    t.stop.assert_not_called();t.move.assert_not_called()
    n._on_command(command(n,identity='new-while-recovering'))
    assert n._result_publisher.publish.call_args.args[0].error_code=='BODY_RECOVERY_IN_PROGRESS'
    # Even if planning recovers, this attempt ends after local neutral return.
    n._health['vlm_node']=(time.monotonic(),True)
    t.body_status.return_value=BodyStatus(4,99,n._odom.stamp_ns,0.,'')
    n._tick()
    assert not n._motion_enabled and n._last_fault=='NODE_UNAVAILABLE:vlm_node'
    assert n._active is None and n._body_after is None
    t.move.assert_not_called();assert t.stop.called


def test_planner_failure_while_held_starts_one_return_and_sensor_loss_still_stops(node):
    from s2e_vlm_robot.sport_client import BodyStatus
    n,t=node;hold_body_look(n,t);t.stop.reset_mock()
    n._health['vlm_node']=(time.monotonic(),False)
    n._tick()
    t.body_return.assert_called_once();t.stop.assert_not_called()
    n._camera_received=0.
    n._tick()
    assert not n._motion_enabled and n._last_fault=='CAMERA_STALE'
    assert not n._body_recovery_reason and t.stop.called


@pytest.mark.parametrize('kind', ['fault', 'disable', 'task'])
def test_body_return_cancellation_discards_queued_movement(node, kind):
    from s2e_vlm_robot.sport_client import BodyStatus
    n, transport = node
    hold_body_look(n, transport)
    n._on_command(command(n, identity='queued-move'))
    if kind == 'fault':
        n._fault('CAMERA_STALE')
    elif kind == 'disable':
        n._set_motion_enabled(NS(data=False), NS())
    else:
        task = NavigationTask()
        task.episode_id = 'replaced'
        n._on_task(task)
    transport.body_status.return_value = BodyStatus(4, 99, n._odom.stamp_ns, 0., '')
    n._tick()
    assert n._body_after is None and n._active is None
    transport.move.assert_not_called()
    transport.stop.assert_called()


def test_body_sweep_cannot_claim_quiescence_while_tilted(node):
    from s2e_vlm_robot.sport_client import BodyStatus
    n, transport = node
    hold_body_look(n, transport)
    n._sweep = sweep(SweepCommand.REQUEST_PAUSE, 1)
    n._executor_paused = True
    n._pause_if_ready()
    assert n._body_returning and not n._quiescent_acked
    transport.body_status.return_value = BodyStatus(4, 99, n._odom.stamp_ns, 0., '')
    n._tick()
    assert n._quiescent_acked


@pytest.mark.parametrize('action,executed', [('look_up', True), ('look_down', False)])
def test_opposite_look_returns_neutral_but_unvalidated_increment_is_not_faked(node, action, executed):
    from s2e_vlm_robot.sport_client import BodyStatus
    n, transport = node
    hold_body_look(n, transport)
    n._on_command(command(n, name=action, identity='next-look'))
    transport.body_status.return_value = BodyStatus(4, 99, n._odom.stamp_ns, 0., '')
    n._tick()
    result = n._result_publisher.publish.call_args.args[0]
    assert result.executed is executed
    if not executed:
        assert result.error_code == 'BODY_LOOK_INCREMENT_UNVALIDATED'
    transport.move.assert_not_called()


def test_body_return_requires_new_robot_origin_pose(node):
    from s2e_vlm_robot.sport_client import BodyStatus
    n, transport = node
    hold_body_look(n, transport)
    n._on_command(command(n, identity='queued-move'))
    transport.body_status.return_value = BodyStatus(4, 99, n._odom.stamp_ns+1_000_000, 0., '')
    n._tick()
    assert n._active is None and n._body_returning
    transport.move.assert_not_called()


def test_intentional_policy_turn_releases_forward_heading(node):
    n, _ = node
    enable(n)
    n._on_command(command(n))
    finish_at_yaw(n, .03)
    n._on_command(command(n, name="turn_left", identity="turn"))
    assert n._translation_reference is None
    finish_at_yaw(n, math.radians(30))
    n._on_command(command(n, identity="forward-after-turn"))
    assert n._active.heading_yaw == pytest.approx(math.radians(30))


@pytest.mark.parametrize("reset", ["disable", "fault", "task"])
def test_heading_is_released_on_execution_reset(node, reset):
    n, _ = node
    enable(n)
    n._on_command(command(n))
    finish_at_yaw(n, .03)
    assert n._translation_reference is not None
    if reset == "disable":
        n._set_motion_enabled(NS(data=False), NS())
    elif reset == "fault":
        n._fault("ODOM_CLOCK_RESET")
    else:
        task = NavigationTask()
        task.episode_id = "new-task"
        n._on_task(task)
    assert n._translation_reference is None


def test_translation_log_matches_transport_command_and_reference(node, monkeypatch):
    from dataclasses import replace
    import json
    n, transport = node
    enable(n)
    n._on_command(command(n))
    finish_at_yaw(n, .03)
    n._on_command(command(n, identity="recorded-step"))
    n._odom = replace(n._odom, yaw=.04)
    logger = Mock()
    monkeypatch.setattr(n, "get_logger", lambda: logger)
    n._tick()
    event = json.loads(logger.info.call_args.args[0].split("robot_motion_event ")[1])
    assert event["event"] == "translate_command"
    assert event["reference_yaw_rad"] == 0.
    assert event["heading_error_rad"] == pytest.approx(-.04)
    assert (event["linear_command_mps"], event["angular_command_radps"]) == transport.move.call_args.args
    assert event["transport_accepted"] is True


def test_startup_inhibited_and_enable_does_not_replay_old_commands(node):
    n, transport = node
    old = command(n)
    n._on_command(old)
    assert n._result_publisher.publish.call_args.args[0].error_code == "MOTION_INHIBITED"
    enable(n)
    n._on_command(old)
    assert n._active is None
    transport.move.assert_not_called()


def test_duplicate_active_action_and_task_replacement(node):
    n, transport = node
    enable(n)
    c = command(n)
    n._on_command(c)
    active = n._active
    n._on_command(c)
    assert n._active is active
    task = NavigationTask()
    task.episode_id = "new-episode"
    n._on_task(task)
    assert n._active is None
    assert n._result_publisher.publish.call_args.args[0].error_code == "TASK_REPLACED"
    transport.stop.assert_called()
    n._on_command(c)
    assert n._active is None


def test_result_waits_for_causal_pose_without_synthetic_collision(node):
    n, _ = node
    enable(n)
    n._on_command(command(n))
    n._last_pose = None
    n._finish(MotionOutput(terminal="OK"))
    n._result_publisher.publish.assert_not_called()
    assert n._pending_result is not None
    pose = StampedPose()
    pose.header.stamp = n.get_clock().now().to_msg()
    pose.header.frame_id = "robot_origin"
    pose.pose.orientation.w = 1.
    pose.status = "OK"
    n._on_pose(pose)
    n._tick()  # Sensor receipt defers result delivery to the control timer.
    result = n._result_publisher.publish.call_args.args[0]
    assert result.executed and not result.collided
    assert result.pose_after.header == pose.header
    assert result.header == pose.header
    # Both actual consumers decode this shared contract after every real step.
    from s2e_vlm_nodes.ros_conversions import habitat_action_result_from_message
    decoded = habitat_action_result_from_message(result)
    assert decoded.executed and not decoded.collided


def test_sensor_loss_during_motion_stops_and_requires_reenable(node):
    n, transport = node
    enable(n)
    n._on_command(command(n))
    n._camera_received = 0
    n._tick()
    assert not n._motion_enabled
    assert n._active is None
    transport.stop.assert_called()
    assert n._last_fault == "CAMERA_STALE"


@pytest.mark.parametrize('action', ['move_forward', 'turn_left', 'turn_right'])
def test_operator_disable_stops_active_motion_and_rejects_late_commands(node, action):
    n, transport = node
    enable(n)
    n._on_command(command(n, name=action))
    n._tick()
    assert n._active is not None
    reply = n._set_motion_enabled(NS(data=False), NS())
    assert reply.success and not n._motion_enabled and n._active is None
    transport.stop.assert_called()
    terminal = n._result_publisher.publish.call_args.args[0]
    assert not terminal.executed and terminal.error_code == 'MOTION_INHIBITED'
    count = transport.move.call_count
    n._on_command(command(n, name=action, identity='late'))
    n._tick()
    assert transport.move.call_count == count


def test_failed_stop_transport_never_certifies_completed_motion(node):
    from s2e_vlm_robot.sport_client import SportClientError
    n, transport = node
    enable(n)
    n._on_command(command(n))
    transport.stop.side_effect = SportClientError('injected stop transport failure')
    n._finish(MotionOutput(terminal='OK'))
    result = n._result_publisher.publish.call_args.args[0]
    assert not result.executed and result.error_code == 'SPORT_STOP_FAILED'
    assert not n._motion_enabled and n._active is None


def test_rotation_settling_sends_stop_until_measured_rest(node):
    from s2e_vlm_robot.motion import MeasuredMotion
    n, transport = node
    enable(n)
    start = n._odom
    n._active = MeasuredMotion(kind="rotate", target=math.radians(60), sample=start,
                               now=time.monotonic(), timeout=9, limits=n._limits)
    # Feed plausible timestamp spacing while keeping receipt/freshness current.
    start_stamp = start.stamp_ns-4_000_000_000
    n._active.start = n._active.last_sample = MotionSample(start_stamp, start.received_s,
                                                         0, 0, 0, 0, 0)
    for i, (angle, rate) in enumerate([(59, .2), (56, -.2), (56, 0)]):
        now = time.monotonic()
        n._odom = MotionSample(start.stamp_ns+i*100_000_000, now, 0, 0,
                               math.radians(angle), 0, rate)
        n._tick()
        assert n._active is not None and n._motion_enabled
        transport.move.assert_not_called()
    assert transport.stop.call_count >= 3
    n._odom = MotionSample(start.stamp_ns+300_000_000, time.monotonic(),
                           0, 0, math.radians(56), 0, 0)
    n._tick()
    assert transport.move.call_args.args == pytest.approx((0, .4))
    assert not n._last_fault


@pytest.mark.parametrize("action,timeout", [("move_forward", 6.), ("turn_left", 7.5), ("turn_right", 7.5)])
def test_fixed_macro_rotation_uses_robot_time_budget(node, action, timeout):
    n, _ = node
    enable(n)
    n._on_command(command(n, name=action))
    assert n._active is not None
    assert n._active.tracking_deadline-n._active.started == pytest.approx(timeout)
    assert n._active.deadline-n._active.started == pytest.approx(timeout + (2. if action != "move_forward" else 0.))


@pytest.mark.parametrize('action,timeout',[('move_forward',6.),('turn_left',15.),('turn_right',15.)])
def test_configured_rotation_minimum_preserves_translation_deadline(node,action,timeout):
    from rclpy.parameter import Parameter
    n,_=node
    n.set_parameters([Parameter('rotate_min_timeout_s',value=15.)])
    enable(n);n._on_command(command(n,name=action))
    assert n._active.tracking_deadline-n._active.started==pytest.approx(timeout)


@pytest.mark.parametrize('executed,reason', [
    (True, 'OK'), (False, 'NO_MOTION_PROGRESS'),
    (False, 'MOTION_INHIBITED'), (False, 'POST_ACTION_POSE_OR_SETTLE_TIMEOUT'),
])
def test_action_result_is_accepted_by_shared_policy_contract(node, executed, reason):
    from s2e_vlm_nodes.ros_conversions import habitat_action_result_from_message
    n, _ = node
    measured_stamp = n._last_pose.header.stamp
    n._publish_result(command(n), executed=executed, status=reason)
    result = n._result_publisher.publish.call_args.args[0]
    decoded = habitat_action_result_from_message(result)
    assert result.header.stamp == measured_stamp == result.pose_after.header.stamp
    assert decoded.executed == executed
    assert decoded.status == ('EXECUTED' if executed else 'FAILED')
    assert decoded.error_code == ('' if executed else reason)
    assert not decoded.collided


@pytest.mark.parametrize('action', ['move_forward', 'turn_left', 'turn_right'])
def test_motion_uses_pose_and_rgb_without_pointcloud_or_slam_health(node, action):
    from sensor_msgs.msg import PointCloud2
    n, transport = node
    # No scan is supplied. A SLAM cloud error must not masquerade as a
    # navigation fault when fresh pose, RGB and policy inputs are available.
    status = NodeStatus()
    status.header.stamp = n.get_clock().now().to_msg()
    status.node_name = 'go2_sensor_input'
    status.is_healthy = False
    status.error_code = 'INSUFFICIENT_LIDAR_POINTS'
    n._on_health(status)
    assert all(s.msg_type is not PointCloud2 for s in n.subscriptions)
    enable(n)
    n._on_command(command(n, name=action))
    n._tick()
    assert n._motion_enabled and n._active is not None
    assert not n._last_fault
    linear, angular = transport.move.call_args.args
    assert linear > 0 if action == 'move_forward' else angular > 0 if action == 'turn_left' else angular < 0
    n._result_publisher.publish.assert_not_called()


def test_late_command_from_superseded_policy_generation_cannot_execute(node):
    n, transport = node
    enable(n)
    n._highest_generation = 2
    n._on_command(command(n))
    assert n._active is None
    assert n._result_publisher.publish.call_args.args[0].error_code == "STALE_POLICY_GENERATION"
    transport.move.assert_not_called()


def test_missing_or_pre_enable_pointgoal_cannot_execute_cached_task(node):
    n, transport = node
    enable(n)
    old_goal = n._pointgoal
    n._pointgoal = None
    n._on_command(command(n))
    n._pointgoal = old_goal
    n._enabled_after = n.get_clock().now().nanoseconds
    n._on_command(command(n, identity="pre-enable-goal"))
    assert n._active is None
    assert n._result_publisher.publish.call_args.args[0].error_code == "POINTGOAL_UNAVAILABLE_OR_STALE"
    transport.move.assert_not_called()


def sweep(kind, sequence):
    c = SweepCommand()
    c.episode_id = "episode"
    c.sweep_id = "sweep"
    c.source_decision_id = "decision"
    c.trajectory_id = "trajectory"
    c.generation = 1
    c.navigation_backend = "pixnav"
    c.command = kind
    c.command_sequence = sequence
    return c


def test_sweep_waits_for_measured_boundary_and_fences_late_actions(node):
    n, _ = node
    enable(n)
    n._on_command(command(n))
    pause = sweep(SweepCommand.REQUEST_PAUSE, 1)
    ack = SweepAck()
    for name in ("episode_id", "sweep_id", "source_decision_id", "trajectory_id", "generation", "command_sequence"):
        setattr(ack, name, getattr(pause, name))
    ack.ack, ack.producer = SweepAck.EXECUTOR_PAUSED, "pixnav_runtime_node"
    n._on_sweep_ack(ack)  # Cross-topic delivery can put the ack first.
    n._on_sweep_command(pause)
    assert not n._quiescent_acked
    n._last_pose = None
    n._finish(MotionOutput(terminal="OK"))
    n._pause_if_ready()
    assert not n._quiescent_acked  # Post-action pose is still pending.
    pose = StampedPose()
    pose.header.stamp = n.get_clock().now().to_msg()
    pose.header.frame_id = "robot_origin"
    pose.pose.orientation.w = 1.
    pose.status = "OK"
    n._on_pose(pose)
    n._tick()  # Sensor receipt defers result delivery to the control timer.
    n._pause_if_ready()
    assert n._ack_publisher.publish.call_args.args[0].ack == SweepAck.CONTROLLER_QUIESCENT
    n._on_command(command(n, identity="late-command"))
    assert n._active is None
    assert n._result_publisher.publish.call_args.args[0].error_code == "SWEEP_OWNS_MOTION"
    n._on_sweep_command(sweep(SweepCommand.START_SWEEP, 2))
    n._on_sweep_command(sweep(SweepCommand.RESUME, 3))
    assert n._ack_publisher.publish.call_args.args[0].ack == SweepAck.CONTROLLER_RELEASED
    n._on_sweep_command(sweep(SweepCommand.APPLY_RESUME, 4))
    assert n._sweep is None


def test_localization_adapter_rejects_origin_reset_during_motion():
    rclpy.init()
    n = RtabPointGoalNode()
    try:
        status = NodeStatus()
        status.state, status.active_mode = "ACTIVE", "IDLE"
        n._on_controller_status(status)
        assert not n._reset_origin(None, NS()).success
        status.state = "INHIBITED"
        n._on_controller_status(status)
        assert n._reset_origin(None, NS()).success
        goal = PointStamped()
        goal.header.frame_id = "map"
        goal.point.x = 5.
        n._on_map_goal(goal)
        first = n._episode_id
        n._on_map_goal(goal)
        assert n._episode_id != first
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('frame', ['', 'odom', 'camera'])
def test_localization_pose_requires_explicit_map_frame(frame):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    rclpy.init()
    n = RtabPointGoalNode()
    try:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = n.get_clock().now().to_msg()
        pose.header.frame_id = frame
        pose.pose.pose.orientation.w = 1.
        n._on_localization_pose(pose)
        assert n._latest_localization_stamp is None
        assert n._session is None
        pose.header.frame_id = 'map'
        n._on_localization_pose(pose)
        assert n._latest_localization_stamp is None  # Unpaired pose is not readiness.
        body=Odometry();body.header.stamp=pose.header.stamp
        body.header.frame_id='odom';body.child_frame_id='base_link'
        body.pose.pose.orientation.w=1.
        n._on_odometry(body)
        assert n._latest_localization_stamp == pose.header.stamp
        assert n._session is not None
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("target_deg,deadline_s,minimum_s", [(30., 7.5,0.), (60., 15.,0.), (30.,15.,15.), (60.,15.,15.)])
def test_real_rotate_action_protocol_finishes_from_odometry(node, target_deg, deadline_s, minimum_s):
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from s2e_vlm_msgs.action import Rotate
    n, transport = node
    from rclpy.parameter import Parameter
    n.set_parameters([Parameter('rotate_min_timeout_s',value=minimum_s)])
    enable(n)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(n)
    client = ActionClient(n, Rotate, "/s2e/controller/rotate")
    try:
        assert client.wait_for_server(timeout_sec=2)
        request = Rotate.Goal()
        request.target_yaw_delta_deg = target_deg
        request.max_yaw_rate_deg_s = 30.
        request.tolerance_deg = 3.
        request.timeout_s = 3.  # Even an old caller cannot shorten the robot budget.
        sent = client.send_goal_async(request)
        executor.spin_until_future_complete(sent, timeout_sec=1)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        started = time.monotonic()
        observed_deadline = None
        while not result.done() and time.monotonic()-started < 2:
            now = time.monotonic()
            yaw = min(math.radians(target_deg), (now-started)*1.0)
            with n._lock:
                if n._active is not None:
                    observed_deadline = n._active.tracking_deadline-n._active.started
                    assert n._active.deadline-n._active.tracking_deadline == pytest.approx(2.)
                stamp = n.get_clock().now().nanoseconds
                n._odom = MotionSample(stamp, now, 0, 0, yaw, 0, 0 if yaw >= math.radians(target_deg) else 1)
                n._camera_received = now
                n._health = {name: (now, True) for name in n._health}
                n._last_pose.header.stamp = n.get_clock().now().to_msg()
            executor.spin_once(timeout_sec=.02)
        assert result.done()
        assert result.result().result.success, result.result().result.result_code
        assert observed_deadline == pytest.approx(deadline_s)
        assert result.result().result.final_yaw_delta_deg == pytest.approx(target_deg, abs=3)
        transport.stop.assert_called()
    finally:
        client.destroy()
        executor.remove_node(n)
        executor.shutdown()


def test_camera_rectification_keeps_capture_stamp_and_rejects_bad_intrinsics():
    rclpy.init()
    n = RtabPointGoalNode()
    try:
        info = CameraInfo()
        info.width, info.height = 32, 24
        info.k = [30., 0., 16., 0., 30., 12., 0., 0., 1.]
        info.d = [.1, 0., 0., 0., 0.]
        info.distortion_model = "plumb_bob"
        n._on_camera_info(info)
        image = Image()
        image.header.stamp = n.get_clock().now().to_msg()
        image.header.frame_id = "original_optical_frame"
        image.width, image.height, image.step = 32, 24, 96
        image.encoding = "bgr8"
        image.data = bytes([10, 20, 30]) * (32*24)
        output = n._rgb_image(image)
        assert output.header.stamp == image.header.stamp
        assert image.header.frame_id == "original_optical_frame"
        assert output.encoding == "rgb8"
        assert n._rectification is not None
        info.k[0] = 0
        n._on_camera_info(info)
        assert n._camera_info is None
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_control_odometry_jump_latches_and_stops_publishing_policy_poses():
    from builtin_interfaces.msg import Time
    from s2e_vlm_robot.goal_geometry import SessionFrame, Pose2D
    rclpy.init()
    n = RtabPointGoalNode()
    try:
        body = TransformStamped()
        body.transform.translation.z = .31
        body.transform.rotation.w = 1.
        n._body_pose_buffer.lookup_transform = Mock(return_value=body)
        n._session = SessionFrame(Pose2D(0, 0, 0))
        n._pose_publisher = Mock()
        assert n._publish_pose(Pose2D(0, 0, 0), Time(sec=1))
        assert not n._publish_pose(Pose2D(1, 0, 0), Time(sec=1, nanosec=100_000_000))
        assert n._pose_fault == "ODOMETRY_JUMP"
        assert not n._publish_pose(Pose2D(1, 0, 0), Time(sec=2))
        assert n._pose_publisher.publish.call_count == 1
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_delayed_odometry_selects_newest_resolvable_image_without_starvation():
    from builtin_interfaces.msg import Time
    from rclpy.parameter import Parameter
    from s2e_vlm_robot.goal_geometry import SessionFrame, Pose2D
    from s2e_vlm_robot.sensor_geometry import stamp_ns
    rclpy.init()
    n = RtabPointGoalNode(parameter_overrides=[Parameter('pose_source',value='odometry')])
    try:
        body = TransformStamped()
        body.transform.translation.z = .31
        body.transform.rotation.w = 1.
        n._body_pose_buffer.lookup_transform = Mock(return_value=body)
        n._session = SessionFrame(Pose2D(0, 0, 0))
        n._localization_received = time.monotonic()
        n._odometry_received = time.monotonic()
        n._history.samples.append((n.get_clock().now().nanoseconds,Pose2D(0,0,0)))
        n._camera_info = CameraInfo()
        n._camera_info.width, n._camera_info.height = 4, 4
        n._image_publisher = Mock()
        n._pose_at = Mock(return_value=None)
        base = n.get_clock().now().nanoseconds-300_000_000
        for i in range(20):
            image = Image()
            stamp = base+i*10_000_000
            image.header.stamp = Time(sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000)
            image.width, image.height, image.step = 4, 4, 12
            image.encoding, image.data = "rgb8", bytes(48)
            n._on_image(image)
        assert len(n._pending_images) == 16
        n._image_publisher.publish.assert_not_called()
        available = base+150_000_000
        n._pose_at = lambda stamp: Pose2D(0, 0, 0) if stamp_ns(stamp) <= available else None
        n._flush_image()
        output = n._image_publisher.publish.call_args.args[0]
        assert stamp_ns(output.header.stamp) == available
        assert len(n._pending_images) == 4
    finally:
        n.destroy_node()
        rclpy.shutdown()
