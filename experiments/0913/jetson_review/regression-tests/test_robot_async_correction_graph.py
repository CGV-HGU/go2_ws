"""Real isolated DDS delivery; synthetic RGB/action ACKs, no robot transport."""
import copy
from dataclasses import replace
from collections import deque
import os
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from s2e_vlm_msgs.msg import (NavigationTask,StampedPose,PointGoal,CameraArticulation,
    HabitatActionCommand,HabitatActionResult,SweepAck,SweepCommand,LocalPolicyResult,LocalPolicyRequest,BoundVlmObservation)
from s2e_vlm_robot.direct_goal_node import DirectGoalNode
from s2e_vlm_robot.direct_goal import direct_goal_decision
from s2e_vlm_core.goal_projection import CameraCalibration
from s2e_vlm_nodes.ros_conversions import vlm_decision_to_message
from s2e_vlm_robot.action_executor_node import ActionExecutorNode
from s2e_vlm_nodes.runtime.pixnav import PixNavRuntimeNode
from s2e_vlm_nodes.runtime.vlm import VlmMockNode
from s2e_vlm_nodes.runtime.sweep_scheduler import SweepSchedulerNode
from s2e_vlm_nodes.sweep_scheduler_node import NODE_CONTRACT as SWEEP_CONTRACT
from s2e_vlm_nodes.pixnav_runtime_node import NODE_CONTRACT
from s2e_vlm_core.sweep_prefetch import SWEEP_CHECKPOINT_REASON
from .test_robot_async_correction_handoff import planner
from .test_pixnav_runtime_node import SequenceWorker,identity
from .test_robot_sweep_checkpoint_feedback import feedback_node
from .test_vlm_runtime_node import _call_vlm_mock
from s2e_vlm_nodes.runtime.vlm import local_policy_result_from_message


@pytest.mark.parametrize('action_ack_delay,episodes',[(.03,1),(.5,5)])
@pytest.mark.parametrize('pause_before_first',[False,True])
def test_real_dds_correction_and_five_episode_resets(monkeypatch,action_ack_delay,episodes,pause_before_first):
    assert os.environ.get('ROS_DOMAIN_ID')=='214'
    with patch.dict(os.environ,{'S2E_RUNTIME_ROLE':'robot_full','S2E_SENSOR_PROFILE':'robot',
        'S2E_SENSOR_CONFIG_DIR':'/work/config/sensors','PIXNAV_PHYSICAL_BODY_LOOK':'true',
        'PIXNAV_FIXED_CAMERA_NOOP':'false','PIXNAV_FIXED_CAMERA_VIEW_ON_LOOK':'false',
        'PIXNAV_FIXED_CAMERA_VIEW_ZOOM':'1','PIXNAV_LOOK_EXECUTION':'forward_0p1',
        'PIXNAV_EXECUTION_MODE':'async_true','PIXNAV_MAX_GO_STEPS':'50',
        'VLM_SWEEP_PREFETCH_MODE':'independent_resume_prefetch',
        'VLM_PAPER_OBSERVATION_TIMING_VARIANT':'adaptive','NAVIGATION_BACKEND':'pixnav'}):
        rclpy.init();worker=SequenceWorker(('look_down','look_up','move_forward','look_down','look_up','stop'))
        worker.backend_identity=identity()
        direct=DirectGoalNode();pixel=PixNavRuntimeNode(NODE_CONTRACT,worker=worker)
        scheduler=SweepSchedulerNode(SWEEP_CONTRACT);io=Node('isolated_correction_io')
    executor=SingleThreadedExecutor()
    for n in (direct,pixel,scheduler,io):executor.add_node(n)
    def pub(cls,topic,qos=qos_profile_sensor_data):return io.create_publisher(cls,topic,qos)
    taskpub=pub(NavigationTask,'/s2e/task',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    imagepub=pub(Image,'/s2e/sensors/camera/image');calpub=pub(CameraInfo,'/s2e/sensors/camera/camera_info')
    posepub=pub(StampedPose,'/s2e/odometry/pose');pgpub=pub(PointGoal,'/s2e/sensors/pointgoal')
    artpub=pub(CameraArticulation,'/s2e/sensors/camera/articulation')
    resultpub=pub(HabitatActionResult,'/s2e/sim/action_result',10)
    ackpub=pub(SweepAck,'/s2e/sweep/ack',10)
    requestpub=pub(SweepCommand,'/s2e/sweep/request',10)
    p=planner(monkeypatch,requestpub.publish);p.get_clock=io.get_clock
    p._start_pending_reference_observation_sweep=Mock()
    p._publish_sweep_ack=lambda kind:VlmMockNode._publish_sweep_ack(p,kind)
    p.sweep_ack_publisher=ackpub;p.last_sensor_bundle=None
    p.vlm_prefetch_guard_condition=Mock();p.sync.gate=NS(can_plan=True)
    # Execute the real controller's pause/quiescence callbacks with an in-memory
    # stop transport. Never construct a SportClient or any motion connection.
    ctrl=NS(_lock=threading.RLock(),_episode_id='',_sweep=None,
        _early_pause_acks=deque(maxlen=32),_executor_paused=False,_quiescent_acked=False,
        _body_command=None,_body_held=False,_body_returning=False,_active=None,
        _pending_result=None,_rotate_reserved=False,_odom=NS(speed=0.,yaw_rate=0.),
        _limits=NS(stop_speed=.03,stop_yaw_rate=.03),_stop=Mock(return_value=''),
        _fault=Mock(),_last_pose=None,_ack_publisher=ackpub,get_clock=io.get_clock,
        _sweep_identity=ActionExecutorNode._sweep_identity)
    ctrl._ack=lambda kind,detail='':ActionExecutorNode._ack(ctrl,kind,detail)
    ctrl._pause_if_ready=lambda:ActionExecutorNode._pause_if_ready(ctrl)
    io.create_subscription(SweepAck,'/s2e/sweep/ack',lambda m:ActionExecutorNode._on_sweep_ack(ctrl,m),10)
    events=[];commands=[];pending=[];view_acks_sent=False;feedback_checks=[];superseded_ack_poses=[]
    original_trace=pixel.runtime.write_handoff_trace
    def trace(event,payload):
        if event=='action_ack_pose_superseded':
            superseded_ack_poses.append(dict(payload))
        original_trace(event,payload)
    pixel.runtime.write_handoff_trace=trace
    def terminal_feedback(message):
        result=local_policy_result_from_message(message)
        if result.action!='stop':return
        assert result.status=='OK' and result.reason==SWEEP_CHECKPOINT_REASON
        feedback,annotations=feedback_node(0.)
        feedback.reference_pending_go_feedback=replace(feedback.reference_pending_go_feedback,
                                                        decision_id=result.decision_id)
        feedback._latest_pose_message().header.stamp=copy.deepcopy(message.header.stamp)
        with patch.object(VlmMockNode,'_queue_no_progress_observation_recovery') as recovery, \
             patch.object(VlmMockNode,'_record_pixnav_navigation_outcome') as outcome:
            assert _call_vlm_mock('_sync_reference_pixnav_feedback',feedback,result)
        feedback._queue_async_blocked_go_observation_recovery.assert_not_called()
        recovery.assert_not_called();outcome.assert_not_called()
        assert annotations[0]['runner_go_execution']['failure_classification']['failure_class']=='not_assessed_async_handoff'
        feedback_checks.append(result.decision_id)
    io.create_subscription(LocalPolicyResult,'/s2e/local_policy/result',terminal_feedback,10)
    def sweep(m):
        events.append((m.episode_id,m.command))
        ActionExecutorNode._on_sweep_command(ctrl,m)
        VlmMockNode._on_sweep_command(p,m)
    io.create_subscription(SweepCommand,'/s2e/sweep/command',sweep,10)
    def request_before_action(request):
        if not pause_before_first:return
        if request.decision_id==corrected_id:return
        eid=p.runtime.task.episode_id
        assert not [c for c in commands if c.episode_id==eid]
        p.reference_async_prefetch_feedback_decision_id=request.decision_id
        VlmMockNode._request_robot_async_correction_pause(p,
            owner_decision_id=request.decision_id,decision_id='correct-'+eid)
    io.create_subscription(LocalPolicyRequest,'/s2e/local_policy/request',request_before_action,10)
    def action(c):
        commands.append(c);ctrl._active=c
        assert direct.sent is not None
        if pause_before_first:
            assert corrected_id and pixel.runtime.active_request.decision_id==corrected_id
            assert (c.episode_id,SweepCommand.APPLY_RESUME) in events
        else:
            p.reference_async_prefetch_feedback_decision_id=direct.sent.decision_id
            VlmMockNode._request_robot_async_correction_pause(p,
                owner_decision_id=direct.sent.decision_id,decision_id='correct-'+c.episode_id)
        pending.append((time.monotonic()+action_ack_delay,c))
    io.create_subscription(HabitatActionCommand,'/s2e/controller/habitat_action',action,10)
    try:
        for episode in range(episodes):
            eid='correction-goal-'+str(episode+1)
            p.runtime.task=NS(episode_id=eid);p.reference_pending_observation_sweep=None
            ctrl._episode_id=eid;view_acks_sent=False
            ctrl._active=None;pending.clear()
            corrected_id=''
            task=NavigationTask(episode_id=eid,scene_id='synthetic-hallway',task_type='PointNav',target_object='pointgoal')
            task.header.stamp=io.get_clock().now().to_msg();taskpub.publish(task)
            deadline=time.monotonic()+12.;next_image=0.
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.002)
                now=time.monotonic()
                for due,c in list(pending):
                    if now<due:continue
                    pending.remove((due,c));ctrl._active=None
                    m=HabitatActionResult();m.header.stamp=io.get_clock().now().to_msg();m.header.frame_id='robot_origin'
                    if action_ack_delay>=.5:
                        # The action finished earlier; delivery waited while
                        # newer independent pose callbacks kept arriving.
                        old_ns=c.header.stamp.sec*10**9+c.header.stamp.nanosec+30_000_000
                        m.header.stamp.sec,m.header.stamp.nanosec=divmod(old_ns,10**9)
                    for key in ('episode_id','command_id','trajectory_id','generation','action'):setattr(m,key,getattr(c,key))
                    m.action_result_id='result-'+c.command_id;m.executed=True;m.status='EXECUTED'
                    m.pose_after.header=m.header;m.pose_after.pose.orientation.w=1.;m.pose_after.pose.position.z=.32
                    resultpub.publish(m);ctrl._pause_if_ready()
                # Hold the first post-decision observation until REQUEST_PAUSE
                # has reached PixelNav. This deterministically exercises the
                # real log's zero-action admission boundary over DDS.
                waiting_for_pause=(pause_before_first and direct.sent is not None
                    and not any(e==eid and kind==SweepCommand.REQUEST_PAUSE for e,kind in events))
                corrected_action_sent=pause_before_first and any(c.episode_id==eid for c in commands)
                if now>=next_image and not waiting_for_pause and not corrected_action_sent:
                    next_image=now+.07
                    image=Image(height=360,width=640,encoding='rgb8',step=1920,data=bytes([100])*640*360*3)
                    image.header.stamp=io.get_clock().now().to_msg();image.header.frame_id='camera'
                    cal=CameraInfo(width=640,height=360,k=[300.,0.,320.,0.,300.,180.,0.,0.,1.]);cal.header=image.header
                    pose=StampedPose();pose.header=copy.deepcopy(image.header);pose.header.frame_id='robot_origin'
                    pose.child_frame_id='base_link';pose.status='OK';pose.pose.position.z=.32;pose.pose.orientation.w=1.
                    pg=PointGoal(episode_id=eid,scene_id=task.scene_id,task_type='PointNav',distance_m=5.,bearing_rad=0.,success_distance_m=1.);pg.header=image.header
                    art=CameraArticulation(episode_id=eid,pitch_deg=0.);art.header=image.header
                    imagepub.publish(image);calpub.publish(cal);artpub.publish(art);pgpub.publish(pg);posepub.publish(pose)
                    # New synthetic views may finish the sweep only after the
                    # real controller + policy boundary ACK barrier has opened.
                    active=p.reference_coordinated_sweep_command
                    if active is not None and active.command==SweepCommand.START_SWEEP and not view_acks_sent:
                        fence=p.reference_pending_observation_sweep.fresh_after_stamp_ns
                        image_ns=image.header.stamp.sec*10**9+image.header.stamp.nanosec
                        assert image_ns>fence
                        assert ctrl._quiescent_acked and ctrl._active is None
                        if pause_before_first:
                            if not corrected_id:
                                d=direct_goal_decision(task=task,pose=pose,goal_xy=(5.,0.),image=image,
                                    calibration=CameraCalibration(640,360,tuple(cal.k)),
                                    extrinsic=direct.extrinsic,now_ns=io.get_clock().now().nanoseconds)
                                corrected_id=d.decision_id
                                direct.bound_pub.publish(BoundVlmObservation(episode_id=eid,
                                    scene_id=task.scene_id,task_type=task.task_type,target_object=task.target_object,
                                    has_pointgoal=True,pointgoal_source_stamp=pg.header.stamp,
                                    pointgoal_distance_m=5.,pointgoal_bearing_rad=0.,pointgoal_success_distance_m=1.,image=image))
                                direct.decision_pub.publish(vlm_decision_to_message(d))
                            if pixel._sweep_deferred_decision is None:continue
                            assert pixel._sweep_deferred_decision.decision_id==corrected_id
                            assert not [c for c in commands if c.episode_id==eid]
                        for kind in (SweepAck.SWEEP_STARTED,SweepAck.VIEW_READY,SweepAck.ANCHOR_RESTORED):
                            p._publish_sweep_ack(kind)
                        view_acks_sent=True
                if (direct.terminal is not None and len(feedback_checks)>episode and view_acks_sent and not scheduler.coordinator.active
                        and pixel._active_sweep_command is None
                        and (not pause_before_first or corrected_action_sent)):break
            assert direct.terminal is not None,(direct.error,pixel.error_code,events)
            assert direct.terminal.reason==SWEEP_CHECKPOINT_REASON
            assert [kind for e,kind in events if e==eid]==[
                SweepCommand.REQUEST_PAUSE,SweepCommand.START_SWEEP,SweepCommand.RESUME,SweepCommand.APPLY_RESUME]
            assert len([c for c in commands if c.episode_id==eid])==1
            assert not scheduler.coordinator.active and ctrl._sweep is None
            if pause_before_first:
                assert pixel.runtime.active_request.decision_id==corrected_id
            else:
                assert pixel.runtime.active_request is None
        assert len(worker.step_calls)==episodes
        assert len(worker.reset_calls)==episodes*(2 if pause_before_first else 1)
        assert len(feedback_checks)==episodes
        if action_ack_delay>=.5 and not pause_before_first:
            assert len(superseded_ack_poses)==episodes
            assert all(p['retained_apply_pose_stamp_ns']>p['action_result_pose_stamp_ns']
                       for p in superseded_ack_poses)
        assert p.vlm_prefetch_guard_condition.trigger.call_count==episodes
        ctrl._fault.assert_not_called()
    finally:
        executor.shutdown()
        for n in (direct,pixel,scheduler,io):n.destroy_node()
        rclpy.shutdown()
