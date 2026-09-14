#!/usr/bin/env python3
"""Freeze one supervised Full or Direct PointGoal trial; never launch ROS."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import yaml

REPO=Path(__file__).resolve().parents[1]
TEMPLATE=REPO/'.local-data/prepared-full-20260912-ordered-pose/run_001'


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare(output,*,distance=None,map_plan=None,fixed_start_plan=None,goal_label=None,localization_run=None,look='forward_0p1',turn_speed_radps=.5,navigation_image='escape-navigation:map-control-20260913',navigation_mode='full'):
    if navigation_mode not in ('full','direct_goal'):raise ValueError('Unsupported navigation mode')
    if look not in ('forward_0p05','forward_0p1','forward_0p2'):raise ValueError('Unsupported look execution')
    if isinstance(turn_speed_radps,bool) or not math.isfinite(turn_speed_radps) or not 0 < turn_speed_radps <= .8:
        raise ValueError('Turn speed must fit the supervised 0.8 rad/s profile limit')
    if sum(x is not None for x in (distance,map_plan,fixed_start_plan))!=1:raise ValueError('Choose one distance, map plan or fixed-start plan')
    if distance is not None and distance not in (5,10):raise ValueError('Prepared straight trials are 5m or 10m')
    map_mode=map_plan is not None
    fixed_mode=fixed_start_plan is not None
    plan=None
    if fixed_mode:
        from fixed_start_goals import validate_plan
        plan=json.loads(Path(fixed_start_plan).read_text())
        if goal_label is None:raise ValueError('Fixed-start trial requires a saved goal label')
        validate_plan(plan,str(goal_label))
    if map_mode:
        plan=json.loads(Path(map_plan).read_text())
        if str(goal_label) not in ('1','2','3','4','5') or not localization_run:raise ValueError('Map trial requires a label and localization run')
        manifest=json.loads((Path(localization_run)/'manifest.json').read_text())
        if plan['source_database_sha256']!=manifest['source_database_sha256']:raise ValueError('Goal and localization map differ')
        if digest(manifest['source_database'])!=plan['source_database_sha256']:raise ValueError('Frozen map changed')
        if plan.get('pose_source')!='localization':raise ValueError('Map plan must use localization')
        selected=[g for g in plan['goals'] if g['label']==str(goal_label)]
        if len(selected)!=1 or selected[0]['message']['header']['frame_id']!='map':raise ValueError('Invalid map goal')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    old=json.loads((TEMPLATE/'manifest.json').read_text())
    for name in old['sha256']:
        source=TEMPLATE/name;target=output/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
    nav=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',navigation_image],text=True).strip()
    for name in ('robot_pointgoal_trial.py','capture_robot_goal.py','fixed_start_goals.py'):
        shutil.copy2(REPO/'scripts'/name,output/'full5m'/name)
    shutil.copy2(REPO/'scripts/robot_recording_integrity.py',output/'robot_recording_integrity.py')
    shutil.copy2(REPO/'scripts/robot_pointgoal_trial.py',output/'full5m/inject_and_observe.py')
    overlay=yaml.safe_load((output/('compose.map-goals.yaml' if map_mode else 'compose.5m.yaml')).read_text())
    overlay['services']['escape']['image']=nav
    overlay['services']['escape']['environment']['PIXNAV_LOOK_EXECUTION']=look
    overlay['services']['escape']['environment']['ROBOT_NAVIGATION_MODE']=navigation_mode
    # Freeze the transport ceiling with this trial's controller setting. The
    # shared default remains intact for older, immutable trial profiles.
    overlay['services']['unitree-command'].setdefault('environment', {})[
        'UNITREE_MAX_ANGULAR_SPEED_RADPS'] = str(max(.6, turn_speed_radps))
    # The mapping input owner stops during odometry-based navigation. Record
    # the navigation owner's corrected cloud as well as the mapping topic.
    from robot_map_session import SLAM_IMAGE
    shutil.copy2(Path('/home/unitree/go2_ws_antarctica/cyclonedds.xml'),output/'recording-cyclonedds.xml')
    overlay['services']['native-recording']=dict(
        image=SLAM_IMAGE,network_mode='host',init=True,restart='no',cpus=2.,
        stop_signal='SIGINT',stop_grace_period='45s',entrypoint=['bash','-lc'],
        environment=dict(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                         CYCLONEDDS_URI='file:///trial/recording-cyclonedds.xml',ROS_LOG_DIR='/trial/native-ros-log'),
        volumes=['${ESCAPE_TRIAL_DIR:?set a fresh trial directory}:/trial'],
        command=['source /opt/ros/jazzy/setup.bash && exec ros2 bag record '
                 '--storage mcap --storage-preset-profile zstd_fast --disable-keyboard-controls '
                 '--max-bag-size 536870912 --output /trial/native-inputs --topics '
                 '/robot_nav/sensors/front_camera/image_raw /robot_nav/sensors/front_camera/camera_info '
                 '/s2e/mapping/cloud /s2e/robot/sensors/cloud /s2e/robot/sensors/odom /s2e/robot/sensors/imu '
                 '/tf /tf_static /rtabmap/localization_pose /rtabmap/info'])
    pose_source='localization' if map_mode else 'odometry'
    overlay['services']['escape']['command']=['bash','-lc',
        'source /opt/ros/jazzy/setup.bash && source /opt/s2e-robot-minimal/setup.bash && '
        'exec ros2 launch s2e_vlm_robot robot_escape.launch.py motion_enabled:=false '
        'navigation_mode:='+navigation_mode+' look_execution:='+look+' pose_source:='+pose_source+
        ' record_model_inputs:=true'+
        ' sensor_input_enabled:='+('false' if map_mode else 'true')+
        f' success_distance_m:=1.0 minimum_rotate_speed_radps:={turn_speed_radps} turn_speed_radps:={turn_speed_radps}'
        ' rotate_settle_window_s:=0.35 rotate_min_timeout_s:=15.0 rotate_recoil_limit_deg:=10.0']
    if navigation_mode=='direct_goal':
        overlay['services']['escape']['command'][-1]+=' direct_goal_max_policy_steps:=500'
    if map_mode:
        overlay['services']['escape']['volumes'] += [str(Path(localization_run).resolve())+':/localization:ro',str(REPO/'.local-data')+':/localization-state:ro']
        shutil.copy2(map_plan,output/'goal-plan.json')
    if fixed_mode:
        shutil.copy2(fixed_start_plan,output/'goal-plan.json')
    (output/'compose.trial.yaml').write_text(yaml.safe_dump(overlay,sort_keys=False))
    duration=360 if map_mode or fixed_mode or distance==10 else 180
    args=('--map-plan /trial/goal-plan.json --goal-label '+str(goal_label)) if map_mode else '--distance-m '+str(distance)
    if fixed_mode:args='--fixed-start-plan /trial/goal-plan.json --goal-label '+str(goal_label)
    wrapper=(TEMPLATE/'run_trial.sh').read_text()
    wrapper=wrapper.replace("for name,digest in m['sha256'].items():", "for name,digest in m['external_config_sha256'].items():\n assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name\nfor name,digest in m['sha256'].items():")
    wrapper=wrapper.replace('compose.5m.yaml','compose.trial.yaml').replace('--distance-m 5 --navigation-mode full',args+' --navigation-mode full --max-duration-s '+str(duration)).replace('240s docker compose',str(duration+60)+'s docker compose')
    wrapper=wrapper.replace('--navigation-mode full','--navigation-mode '+navigation_mode)
    wrapper=wrapper.replace('# Physical Full 5m run.','# Physical Full PointGoal run.')
    wrapper=wrapper.replace('dc up -d --no-deps unitree-command pixnav escape',
                            'dc up -d --no-deps native-recording unitree-command pixnav escape')
    wrapper=wrapper.replace('  stop_code=$?', '  stop_code=$?\n  dc stop native-recording\n  native_stop_code=$?\n  dc logs --no-color native-recording > "$trial_root/native-recording.log" 2>&1')
    wrapper=wrapper.replace('touch "$trial_root/STACK_STARTED"',
                            '[[ -n "$(dc ps --status running -q native-recording)" ]] || exit 1\n'
                            'touch "$trial_root/STACK_STARTED"')
    wrapper=wrapper.replace('if [[ -f "$trial_root/CANCEL" ]]; then exit 1; fi',
                            'if [[ -f "$trial_root/CANCEL" ]]; then exit 1; fi\n'
                            '[[ -n "$(dc ps --status running -q native-recording)" ]] || exit 1\n'
                            'dc logs --no-color native-recording > "$trial_root/native-recording.log" 2>&1\n'
                            'python3 "$trial_root/robot_recording_integrity.py" --run-dir "$trial_root" --live '
                            '--output "$trial_root/recording-preflight.json" > "$trial_root/recording-preflight.log"')
    wrapper=wrapper.replace('  if [[ "$stop_code" -ne 0 ]]; then',
                            '  observer_running="$(docker container inspect --format \'{{.State.Running}}\' "$observer_name" 2>/dev/null || true)"\n'
                            '  if [[ "$stop_code" -eq 0 && "$native_stop_code" -eq 0 && "$observer_running" != true ]]; then touch "$trial_root/RECORDERS_CLOSED"; fi\n'
                            '  python3 "$trial_root/robot_recording_integrity.py" --run-dir "$trial_root" '
                            '--output "$trial_root/recording-integrity.json" > "$trial_root/recording-check.log" 2>&1\n'
                            '  if [[ "$?" -ne 0 ]]; then touch "$trial_root/RECORDING_INCOMPLETE"; fi\n'
                            '  if [[ "$stop_code" -ne 0 ]]; then')
    (output/'run_trial.sh').write_text(wrapper)
    for name in ('compose.5m.yaml','compose.map-goals.yaml','five_meter_operator.py'):
        (output/name).unlink()
    m=dict(images=dict(old['images'],escape=nav,**{'native-recording':SLAM_IMAGE}),runtime_conditions=dict(navigation=navigation_mode,execution='async_true',look_execution=look,goal_frame='map' if map_mode else 'odom',goal_distance_m=distance,goal_label=goal_label,success_radius_m=1.,initial_motion_enabled=False,max_duration_s=duration,
           turn_speed_radps=turn_speed_radps,minimum_rotate_speed_radps=turn_speed_radps,rotate_settle_window_s=.35,rotate_min_timeout_s=15.,rotate_recoil_limit_deg=10.),
           physical_drive_validated=False,offline_checks_passed=False,ready_for_supervised_trial=False,
           baseline_validation=old['validation_result'],campaign_validation=str(REPO/'.local-data/jetson-goal-campaign-audit-20260912/results.json'),
           checkpoint=old['checkpoint'],camera_time_quality=old['camera_time_quality'],
           final_yaw_requested=False,look_distance_selection='10cm has physical baseline evidence; 5/20cm are explicit unvalidated comparison settings.')
    m['native_input_archive']=str(output/'native-inputs')
    m['native_recording_scope']='Common read-only camera/cloud/odom/IMU/TF bag; verify in-drive topic coverage after every episode.'
    m['runtime_conditions']['record_model_inputs']=True
    m['runtime_conditions']['forward_speed_mps']=.5
    m['runtime_conditions']['observation_turn_speed_radps']=turn_speed_radps
    if fixed_mode:
        m['runtime_conditions'].update(localization_method='fixed_start_odometry',fixed_start_origin_id=plan['origin']['origin_id'],requires_marked_start_before_each_episode=True,icp_corrections_used=False)
    if navigation_mode=='direct_goal':
        m['runtime_conditions'].update(direct_goal_max_policy_steps=500,vlm_calls_expected=0,full_observation_sweeps_enabled=False)
    m['sha256']={str(p.relative_to(output)):digest(p) for p in output.rglob('*') if p.is_file()}
    m['external_config_sha256']={str(REPO/name):digest(REPO/name) for name in ('compose.robot-minimal.yaml','compose.robot-full.yaml','config/robot-full.env')}
    (output/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True,type=Path)
    t=p.add_mutually_exclusive_group(required=True);t.add_argument('--distance-m',type=int,choices=(5,10));t.add_argument('--map-plan',type=Path);t.add_argument('--fixed-start-plan',type=Path)
    p.add_argument('--goal-label',choices=('1','2','3','4','5'));p.add_argument('--localization-run',type=Path)
    p.add_argument('--look',choices=('forward_0p05','forward_0p1','forward_0p2'),default='forward_0p1')
    p.add_argument('--turn-speed-radps',type=float,default=.5)
    p.add_argument('--navigation-image',default='escape-navigation:map-control-20260913')
    p.add_argument('--navigation-mode',choices=('full','direct_goal'),default='full')
    a=p.parse_args();print(json.dumps(prepare(a.output,distance=a.distance_m,map_plan=a.map_plan,fixed_start_plan=a.fixed_start_plan,goal_label=a.goal_label,localization_run=a.localization_run,look=a.look,turn_speed_radps=a.turn_speed_radps,navigation_image=a.navigation_image,navigation_mode=a.navigation_mode),indent=2))


if __name__=='__main__':main()
