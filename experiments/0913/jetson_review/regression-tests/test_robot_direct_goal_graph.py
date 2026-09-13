"""Real isolated ROS graph with synthetic sensors/policy and no robot transport."""
import copy
import math
import os
import time
from unittest.mock import patch
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from s2e_vlm_msgs.msg import (NavigationTask,StampedPose,PointGoal,CameraArticulation,
    HabitatActionCommand,HabitatActionResult,VlmDecision)
from s2e_vlm_robot.direct_goal_node import DirectGoalNode
from s2e_vlm_nodes.runtime.pixnav import PixNavRuntimeNode
from s2e_vlm_nodes.pixnav_runtime_node import NODE_CONTRACT
from .test_pixnav_runtime_node import SequenceWorker,identity


@pytest.mark.parametrize('mode,goal_xy,expected,reason',[
    ('physical',(5.,0.),['look_down','look_up'],'BODY_LOOK_LIMIT_REACHED'),
    ('forward_0p1',(5.,0.),['move_forward','move_forward'],''),
    ('forward_0p1',(3.96820189446,-2.77232200650),['move_forward','move_forward'],''),
    ('forward_0p1',(2.84914973007,1.25823633873),['move_forward','move_forward'],''),
])
def test_direct_goal_one_session_through_real_ros_callbacks(mode,goal_xy,expected,reason):
    assert os.environ.get('ROS_DOMAIN_ID')=='214'
    with patch.dict(os.environ,{'S2E_RUNTIME_ROLE':'robot_full','S2E_SENSOR_PROFILE':'robot',
        'S2E_SENSOR_CONFIG_DIR':'/work/config/sensors','PIXNAV_PHYSICAL_BODY_LOOK':'true',
        'PIXNAV_FIXED_CAMERA_NOOP':'false','PIXNAV_FIXED_CAMERA_VIEW_ON_LOOK':'false',
        'PIXNAV_FIXED_CAMERA_VIEW_ZOOM':'1','PIXNAV_LOOK_EXECUTION':mode,
        'PIXNAV_EXECUTION_MODE':'async_true','PIXNAV_MAX_GO_STEPS':'500'}):
        rclpy.init();worker=SequenceWorker(('look_down','look_down','stop'));worker.backend_identity=identity()
        direct=DirectGoalNode();pixel=PixNavRuntimeNode(NODE_CONTRACT,worker=worker);io=Node('direct_goal_synthetic_io')
    executor=SingleThreadedExecutor()
    for n in (direct,pixel,io):executor.add_node(n)
    def pub(cls,topic,qos=qos_profile_sensor_data):return io.create_publisher(cls,topic,qos)
    taskpub=pub(NavigationTask,'/s2e/task',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    imagepub=pub(Image,'/s2e/sensors/camera/image');calpub=pub(CameraInfo,'/s2e/sensors/camera/camera_info')
    posepub=pub(StampedPose,'/s2e/odometry/pose');pgpub=pub(PointGoal,'/s2e/sensors/pointgoal')
    artpub=pub(CameraArticulation,'/s2e/sensors/camera/articulation');resultpub=pub(HabitatActionResult,'/s2e/sim/action_result',10)
    commands=[];decisions=[];x=0.;pitch=0.
    io.create_subscription(VlmDecision,'/s2e/vlm/decision',decisions.append,10)
    def on_command(c):
        nonlocal x,pitch
        commands.append(c)
        if c.action=='move_forward':x+=c.forward_distance_m
        elif c.action=='look_down':pitch=.26
        elif c.action=='look_up':pitch=0.
        else:raise AssertionError(c.action)
        m=HabitatActionResult();m.header.stamp=io.get_clock().now().to_msg();m.header.frame_id='robot_origin'
        for key in ('episode_id','command_id','trajectory_id','generation','action'):setattr(m,key,getattr(c,key))
        m.action_result_id='result-'+c.command_id;m.habitat_step_index=len(commands);m.executed=True;m.status='EXECUTED'
        m.pose_after.header=m.header;m.pose_after.pose.position.x=x;m.pose_after.pose.position.z=.32
        m.pose_after.pose.orientation.y=math.sin(pitch/2);m.pose_after.pose.orientation.w=math.cos(pitch/2)
        resultpub.publish(m)
    io.create_subscription(HabitatActionCommand,'/s2e/controller/habitat_action',on_command,10)
    try:
        task=NavigationTask(episode_id='isolated-direct',scene_id='hallway',task_type='PointNav',target_object='pointgoal')
        task.header.stamp=io.get_clock().now().to_msg();taskpub.publish(task)
        deadline=time.monotonic()+12.;next_image=0.
        while direct.terminal is None and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.002)
            if time.monotonic()<next_image:continue
            next_image=time.monotonic()+.07
            image=Image(height=360,width=640,encoding='rgb8',step=1920,data=bytes([100])*640*360*3)
            image.header.stamp=io.get_clock().now().to_msg();image.header.frame_id='camera'
            cal=CameraInfo(width=640,height=360,k=[300.,0.,320.,0.,300.,180.,0.,0.,1.]);cal.header=image.header
            pose=StampedPose();pose.header=copy.deepcopy(image.header);pose.header.frame_id='robot_origin';pose.child_frame_id='base_link';pose.status='OK'
            pose.pose.position.x=x;pose.pose.position.z=.32;pose.pose.orientation.y=math.sin(pitch/2);pose.pose.orientation.w=math.cos(pitch/2)
            pg=PointGoal(episode_id=task.episode_id,scene_id=task.scene_id,task_type='PointNav',distance_m=math.hypot(goal_xy[0]-x,goal_xy[1]),bearing_rad=math.atan2(goal_xy[1],goal_xy[0]-x),success_distance_m=1.);pg.header=image.header
            art=CameraArticulation(episode_id=task.episode_id,pitch_deg=0.);art.header=image.header
            # Deliver independent topics; policy waits for actual matching stamps.
            imagepub.publish(image);calpub.publish(cal);artpub.publish(art);pgpub.publish(pg);posepub.publish(pose)
        assert direct.terminal is not None,(direct.error,pixel.state,pixel.error_code,pixel.status_message)
        assert [c.action for c in commands]==expected
        assert direct.terminal.reason==reason
        assert len(decisions)==1 and len(worker.reset_calls)==1
        assert direct.goal==pytest.approx(goal_xy)
        if mode=='forward_0p1':assert [c.forward_distance_m for c in commands]==pytest.approx([.1,.1])
        else:assert x==0.
    finally:
        executor.shutdown()
        for n in (direct,pixel,io):n.destroy_node()
        rclpy.shutdown()
