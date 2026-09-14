"""Execute the prepared operator with real ROS message types, fake I/O and time.

Run only in an isolated navigation container. No ROS node is initialized.
"""
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace as NS

import pytest
import rclpy
import rclpy.node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from s2e_vlm_msgs.msg import NavigationTask, NodeStatus, PointGoal, StampedPose
from s2e_vlm_robot import sport_client


@pytest.fixture
def run_trial(monkeypatch, tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '214'
    assert set(os.listdir('/sys/class/net')) == {'lo'}
    script = Path(__file__).with_name('robot_pointgoal_trial.py')
    source = script.read_text().replace("Path('/trial/full5m/runtime')", repr_path := 'Path('+repr(str(tmp_path))+')')
    source = source.replace("Path('/trial/START_GOAL')", 'Path('+repr(str(tmp_path/'START_GOAL'))+')')
    assert repr_path in source

    def run(scenario, mode='full', distance=5., map_mode=False, fixed_mode=False, duration=None):
        state = NS(t=10., enabled=False, published=None, sent=0., callbacks={},
                   calls=[], direct_stops=0, publisher_calls=0)
        if scenario == 'stop_before_start':
            (tmp_path/'STOP').touch()

        def stamp():
            sec, ns = divmod(int((1_000_000+state.t)*1e9), 10**9)
            return Time(sec=sec, nanosec=ns)

        def emit(topic, msg):
            for callback in state.callbacks.get(topic, []):
                callback(msg)

        def spin(*_, **__):
            state.t += 61. if scenario == 'episode_timeout' and state.published else .1
            odom = Odometry()
            odom.header.stamp = stamp(); odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.z = math.sin(math.pi/4)
            odom.pose.pose.orientation.w = math.cos(math.pi/4)
            odom.pose.pose.position.x = 10.; odom.pose.pose.position.y = 20.
            if state.published and scenario == 'success':
                odom.pose.pose.position.y = 20.+distance-.1
                if fixed_mode: odom.pose.pose.position.x = state.published.point.x
            emit('/s2e/robot/sensors/odom', odom)
            control=StampedPose();control.header.stamp=stamp()
            control.header.frame_id='robot_origin';control.status='OK'
            emit('/s2e/odometry/pose',control)
            if map_mode:
                loc=PoseWithCovarianceStamped();loc.header.stamp=stamp();loc.header.frame_id='map'
                loc.pose.pose.orientation.w=1.;loc.pose.pose.position.x=-30.;loc.pose.pose.position.y=100.
                if state.published and scenario=='success': loc.pose.pose.position.x=-25.1
                if state.published and scenario=='stale_map_pose': loc.header.stamp=Time(sec=1)
                if scenario=='wrong_localization_frame': loc.header.frame_id='odom'
                emit('/rtabmap/localization_pose',loc)
                if state.published and scenario=='map_changed':
                    pointer=json.loads((tmp_path/'current-localization.json').read_text())
                    pointer['source_database_sha256']='changed'
                    (tmp_path/'current-localization.json').write_text(json.dumps(pointer))
            for name in ('go2_action_executor','vlm_node','direct_goal_pixnav','pixnav_runtime_node',
                         'sweep_scheduler_node','rtab_pointgoal_adapter'):
                if mode=='direct_goal' and name in ('vlm_node','sweep_scheduler_node'):continue
                if scenario=='missing_full_sweep' and name=='sweep_scheduler_node':continue
                msg = NodeStatus(); msg.node_name = name; msg.header.stamp = stamp()
                msg.is_healthy = True; msg.active_mode = 'IDLE'
                msg.state = 'IDLE' if state.enabled else 'INHIBITED'
                if name=='direct_goal_pixnav' and state.published and scenario!='episode_timeout':
                    msg.state='TERMINAL'
                emit('/s2e/controller/status', msg)
            pg = PointGoal(); pg.header.stamp = stamp(); pg.episode_id = 'previous-goal'
            pg.distance_m = .1 if scenario in ('old_goal_only','success','map_pose_not_at_goal','stale_map_pose') else 5.
            pg.success_distance_m = .5
            if state.published and scenario in ('success','map_pose_not_at_goal','stale_map_pose'):
                task = NavigationTask(); task.header.stamp = stamp()
                task.episode_id = 'new-goal'; task.task_type = 'PointNav'
                emit('/s2e/task', task); pg.episode_id = task.episode_id
            emit('/s2e/sensors/pointgoal', pg)
            if state.published and scenario not in ('success','episode_timeout') and state.t-state.sent > .5:
                (tmp_path/'STOP').touch()
            if scenario == 'enable_timeout':
                (tmp_path/'STOP').touch() if state.calls else None

        class Client:
            def wait_for_service(self, **_):
                return not (scenario == 'disable_unavailable' and state.published)

            def call_async(self, req):
                state.calls.append(req.data)
                state.enabled = req.data
                ok = not (scenario == 'enable_timeout' and req.data)
                return NS(done=lambda:ok, result=lambda:NS(success=True,message='test'))

        class Node:
            def __init__(self, *_): pass
            def create_subscription(self, cls, topic, callback, qos):
                state.callbacks.setdefault(topic, []).append(callback)
            def create_client(self, cls, service):
                if service.endswith('/get_parameters'):
                    def params(req):
                        values={'pub_loc_pose_only_when_localizing':True,'Mem/IncrementalMemory':'false',
                                'RGBD/LinearUpdate':'0','RGBD/AngularUpdate':'0','pose_source':'localization'}
                        if fixed_mode: values['pose_source']='odometry'
                        if scenario=='fixed_wrong_pose_source': values['pose_source']='localization'
                        if scenario=='wrong_native_parameters': values['pub_loc_pose_only_when_localizing']=False
                        response=NS(values=[NS(type=1 if isinstance(values[k],bool) else 4,bool_value=values[k],string_value=values[k]) for k in req.names])
                        return NS(done=lambda:True,result=lambda:response)
                    return NS(wait_for_service=lambda **_:True,call_async=params)
                return Client()
            def count_publishers(self, topic): return 2 if scenario=='duplicate_sensor_owner' else 1
            def create_publisher(self, *_):
                def publish(msg):
                    state.published = msg; state.sent = state.t; state.publisher_calls += 1
                return NS(publish=publish, get_subscription_count=lambda:1)
            def get_clock(self):
                return NS(now=lambda:NS(nanoseconds=int((1_000_000+state.t)*1e9),to_msg=stamp))
            def destroy_node(self): pass

        def direct_stop(): state.direct_stops += 1
        monkeypatch.setattr(sport_client, 'SportClient', lambda *a, **k:NS(stop=direct_stop))
        monkeypatch.setattr(time, 'monotonic', lambda:state.t)
        monkeypatch.setattr(time, 'time', lambda:1_000_000+state.t)
        monkeypatch.setattr(rclpy.node, 'Node', Node)
        monkeypatch.setattr(rclpy, 'init', lambda **_:None)
        monkeypatch.setattr(rclpy, 'shutdown', lambda:None)
        monkeypatch.setattr(rclpy, 'ok', lambda:False)
        monkeypatch.setattr(rclpy, 'spin_once', spin)
        monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *a, **k:None)
        monkeypatch.setattr(signal, 'signal', lambda *a:None)
        argv=['trial','--distance-m',str(distance),'--navigation-mode',mode]
        if map_mode:
            manifest=dict(path=str(tmp_path),status='running',host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),source_database_sha256='map-hash')
            if scenario=='stopped_run': manifest['status']='stopped'
            (tmp_path/'manifest.json').write_text(json.dumps(manifest))
            (tmp_path/'current-localization.json').write_text(json.dumps(manifest))
            plan=dict(pose_source='localization',source_database_sha256='map-hash',goals=[dict(label=str(i),topic='/s2e/goal/map',message=dict(header=dict(frame_id='map'),point=dict(x=-25.,y=100.,z=0.))) for i in range(1,6)])
            if scenario=='wrong_map': plan['source_database_sha256']='other-map'
            (tmp_path/'plan.json').write_text(json.dumps(plan))
            argv=['trial','--map-plan',str(tmp_path/'plan.json'),'--goal-label','3','--localization-run',str(tmp_path),'--localization-state-root',str(tmp_path)]
        if fixed_mode:
            from fixed_start_goals import make_origin, capture_goal, make_plan
            from test_fixed_start_goals import snapshot
            origin=make_origin(snapshot(),100.,'boot-a')
            goal=capture_goal(origin,'1',snapshot(x=8.,y=25.,now=101.),101.,'boot-a')
            plan=make_plan(origin,[goal])
            confirmation=dict(origin_id=origin['origin_id'],fixed_start_confirmed=True,confirmed_unix=time.time())
            if scenario=='fixed_old_confirmation': confirmation['confirmed_unix']-=301
            if scenario=='fixed_wrong_origin': confirmation['origin_id']='another-start'
            if scenario=='fixed_near_goal': plan['goals'][0]['point_goal_xy_m']=dict(x=.2,y=.1)
            (tmp_path/'START_GOAL').write_text(json.dumps(confirmation))
            (tmp_path/'plan.json').write_text(json.dumps(plan))
            argv=['trial','--fixed-start-plan',str(tmp_path/'plan.json'),'--goal-label','1','--navigation-mode',mode]
        if duration is not None:
            argv.extend(['--max-duration-s',str(duration)])
        monkeypatch.setattr(sys, 'argv', argv)
        code = 0
        try:
            exec(compile(source, str(script), 'exec'), {'__name__':'__main__'})
        except SystemExit as error:
            code = error.code
        return state, json.loads((tmp_path/'result.json').read_text()), code
    return run


def test_stop_before_start_never_enables_or_publishes_goal(run_trial):
    state, result, code = run_trial('stop_before_start')
    assert True not in state.calls and state.publisher_calls == 0
    assert not result['goal_published'] and code != 0


def test_old_goal_arrival_cannot_complete_new_five_meter_trial(run_trial):
    state, result, code = run_trial('old_goal_only')
    assert state.publisher_calls == 1
    assert result['reason'] != 'GOAL_DISTANCE_REACHED' and code != 0
    assert state.calls[-1] is False


def test_current_heading_goal_and_matching_fresh_arrival_complete(run_trial):
    state, result, code = run_trial('success')
    assert state.published.point.x == pytest.approx(10.)
    assert state.published.point.y == pytest.approx(25.)
    assert result['reason'] == 'GOAL_DISTANCE_REACHED' and code == 0
    assert state.calls == [True, False]


def test_direct_policy_terminal_is_not_goal_success(run_trial):
    state,result,code=run_trial('direct_terminal','direct_goal')
    assert result['reason']=='PIXNAV_TERMINAL_BEFORE_GOAL' and code!=0
    assert state.calls==[True,False] and state.direct_stops==1


def test_direct_arrival_takes_priority_over_policy_terminal(run_trial):
    state,result,code=run_trial('success','direct_goal')
    assert result['reason']=='GOAL_DISTANCE_REACHED' and code==0
    assert state.calls==[True,False]


@pytest.mark.parametrize('mode', ['full','direct_goal'])
def test_thirty_minute_episode_survives_old_limits_then_stops_and_records(run_trial, mode):
    state,result,code=run_trial('episode_timeout',mode,fixed_mode=True,duration=1800)
    assert 1800 < state.t-state.sent <= 1861
    assert result['reason']=='TRIAL_TIME_LIMIT' and result['max_duration_s']==1800
    assert result['goal_published'] and result['shutdown_service_confirmed']
    assert state.calls==[True,False] and state.direct_stops==1 and code!=0


def test_longer_limit_preserves_direct_model_stop(run_trial):
    state,result,code=run_trial('direct_terminal','direct_goal',duration=1800)
    assert result['reason']=='PIXNAV_TERMINAL_BEFORE_GOAL'
    assert state.t-state.sent < 1 and result['max_duration_s']==1800
    assert state.calls==[True,False] and state.direct_stops==1 and code!=0


@pytest.mark.parametrize('scenario', ['disable_unavailable','enable_timeout'])
def test_transport_stop_is_attempted_even_when_motion_service_fails(run_trial, scenario):
    state, result, code = run_trial(scenario)
    assert state.direct_stops >= 1 and code != 0
    if scenario == 'enable_timeout':
        assert state.publisher_calls == 0

@pytest.mark.parametrize('distance',[5.,10.])
def test_five_and_ten_meter_goal_distance(run_trial,distance):
    state,result,code=run_trial('success',distance=distance)
    assert code==0
    assert state.published.point.y==pytest.approx(20.+distance)
    assert result['success_distance_m']==.5


def test_saved_map_goal_uses_map_pose_despite_different_odometry(run_trial):
    state,result,code=run_trial('success',map_mode=True)
    assert code==0 and result['goal_frame']=='map' and result['goal_label']=='3'
    assert state.published.header.frame_id=='map'
    assert state.published.point.x==-25. and state.published.point.y==100.
    assert state.calls==[True,False]

@pytest.mark.parametrize('scenario',['wrong_map','stopped_run','wrong_localization_frame','wrong_native_parameters','duplicate_sensor_owner'])
def test_invalid_map_setup_never_enables(run_trial,scenario):
    state,result,code=run_trial(scenario,map_mode=True)
    assert code!=0 and state.publisher_calls==0 and True not in state.calls

@pytest.mark.parametrize('scenario',['old_goal_only','map_pose_not_at_goal','stale_map_pose','map_changed'])
def test_map_arrival_requires_same_map_and_current_actual_pose(run_trial,scenario):
    state,result,code=run_trial(scenario,map_mode=True)
    assert code!=0 and result['reason']!='GOAL_DISTANCE_REACHED'
    assert state.calls==[True,False] and state.direct_stops==1


@pytest.mark.parametrize('mode',['full','direct_goal'])
def test_fixed_start_saved_off_axis_goal_uses_current_boot_odom_and_stops(run_trial,mode):
    state,result,code=run_trial('success',mode=mode,fixed_mode=True)
    assert code==0 and state.publisher_calls==1
    assert state.published.header.frame_id=='odom'
    assert state.published.point.x==pytest.approx(8.)
    assert state.published.point.y==pytest.approx(25.)
    assert result['localization_method']=='fixed_start_odometry'
    assert result['physical_arrival_verified'] is False
    assert state.calls==[True,False] and state.direct_stops==1


@pytest.mark.parametrize('scenario',['fixed_old_confirmation','fixed_wrong_origin','fixed_near_goal',
                                     'fixed_wrong_pose_source','duplicate_sensor_owner'])
def test_fixed_start_invalid_binding_or_control_source_never_enables(run_trial,scenario):
    state,result,code=run_trial(scenario,fixed_mode=True)
    assert code!=0 and state.publisher_calls==0 and True not in state.calls


def test_full_still_requires_its_sweep_scheduler_before_motion(run_trial):
    state,result,code=run_trial('missing_full_sweep')
    assert code!=0 and state.publisher_calls==0 and True not in state.calls
