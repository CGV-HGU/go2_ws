#!/usr/bin/env python3
"""Check a prepared trial against tested code/configuration without starting it."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

REPO=Path(__file__).resolve().parents[1]


def validate(root,results):
    root=Path(root).resolve();result=json.loads(Path(results).read_text());m=json.loads((root/'manifest.json').read_text())
    if not result.get('offline_checks_passed'):raise ValueError('Campaign validation is incomplete')
    if m['images']['escape']!=result['navigation_image_id']:raise ValueError('Different navigation image')
    if any((root/name).exists() for name in ('USED','START_GOAL','STACK_STARTED')):raise ValueError('Trial has already been used')
    for name,sha in m['external_config_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=sha:raise ValueError('External configuration changed: '+name)
    for name,sha in m['sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=sha:raise ValueError('Prepared artifact changed: '+name)
    for name,source in [('full5m/inject_and_observe.py','scripts/robot_pointgoal_trial.py'),('full5m/capture_robot_goal.py','scripts/capture_robot_goal.py')]:
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=result['source_sha256'][source]:raise ValueError('Operator differs from the checked code')
    helper=root/'full5m/fixed_start_goals.py'
    if helper.exists() and hashlib.sha256(helper.read_bytes()).hexdigest()!=result['source_sha256'].get('scripts/fixed_start_goals.py'):
        raise ValueError('Fixed-start helper differs from the checked code')
    recorder=root/'robot_recording_integrity.py'
    if recorder.exists() and hashlib.sha256(recorder.read_bytes()).hexdigest()!=result['source_sha256'].get('scripts/robot_recording_integrity.py'):
        raise ValueError('Recording checker differs from the checked code')
    env=dict(os.environ,ESCAPE_TRIAL_DIR=str(root))
    dc=['docker','compose','--env-file',str(REPO/'config/robot-full.env'),'-f',str(REPO/'compose.robot-minimal.yaml'),'-f',str(REPO/'compose.robot-full.yaml'),'-f',str(root/'compose.trial.yaml')]
    rendered=json.loads(subprocess.check_output(dc+['config','--format','json'],env=env,text=True))
    services=rendered['services'];conditions=m['runtime_conditions']
    for name,image in m['images'].items():
        if services[name]['image']!=image:raise ValueError('Image mismatch: '+name)
        if subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',image],text=True).strip()!=image:raise ValueError('Image unavailable: '+name)
    for name in ('escape','pixnav'):
        e=services[name]['environment']
        if e['PIXNAV_ALLOWED_ACTIONS']!='stop,move_forward,turn_left,turn_right,look_up,look_down':raise ValueError('Raw policy action mask changed')
        if e['PIXNAV_FIXED_CAMERA_NOOP']!='false' or e['PIXNAV_FIXED_CAMERA_VIEW_ON_LOOK']!='false':raise ValueError('Unexpected camera workaround')
    e=services['escape']['environment']
    if e['PIXNAV_LOOK_EXECUTION']!=conditions['look_execution'] or e['ROBOT_MOTION_ENABLED']!='false':raise ValueError('Motion/look configuration mismatch')
    command=' '.join(services['escape'].get('command') or [])
    launch_values=dict(token.split(':=',1) for token in shlex.split(command) if ':=' in token)
    mode=conditions.get('navigation','full')
    if mode not in ('full','direct_goal') or launch_values.get('navigation_mode')!=mode:
        raise ValueError('Planner differs from the prepared comparison method')
    if e.get('ROBOT_NAVIGATION_MODE')!=mode:
        raise ValueError('Planner environment differs from the prepared comparison method')
    if conditions.get('record_model_inputs') is True and launch_values.get('record_model_inputs')!='true':
        raise ValueError('Full model input recording is not enabled')
    if '--navigation-mode '+mode not in (root/'run_trial.sh').read_text():
        raise ValueError('Operator and planner comparison modes differ')
    if mode=='direct_goal' and int(launch_values.get('direct_goal_max_policy_steps','0'))!=conditions['direct_goal_max_policy_steps']:
        raise ValueError('Direct final-goal session budget differs from the prepared condition')
    for key in ('turn_speed_radps','minimum_rotate_speed_radps','rotate_settle_window_s',
                'rotate_min_timeout_s','rotate_recoil_limit_deg'):
        if key in conditions and float(launch_values.get(key,'nan'))!=conditions[key]:
            raise ValueError('Rotation setting differs from the prepared condition: '+key)
    if float(launch_values.get('success_distance_m','1.0'))!=conditions['success_radius_m']:
        raise ValueError('Arrival radius differs from the prepared condition')
    if conditions['goal_frame']=='map':
        if 'pose_source:=localization' not in command or 'sensor_input_enabled:=false' not in command:raise ValueError('Map trial must use external localization sensor owner')
    elif 'pose_source:=odometry' not in command or 'sensor_input_enabled:=true' not in command:
        raise ValueError('Straight trial must own its odometry input')
    if conditions.get('localization_method')=='fixed_start_odometry':
        from fixed_start_goals import validate_plan
        plan=json.loads((root/'goal-plan.json').read_text());validate_plan(plan,conditions['goal_label'])
        if conditions['goal_frame']!='odom' or conditions.get('icp_corrections_used') is not False or conditions.get('fixed_start_origin_id')!=plan['origin']['origin_id']:
            raise ValueError('Fixed-start trial coordinate contract changed')
        if '--fixed-start-plan /trial/goal-plan.json' not in (root/'run_trial.sh').read_text():
            raise ValueError('Fixed-start operator is not selected')
    subprocess.run(['bash','-n',str(root/'run_trial.sh')],check=True)
    m.update(offline_checks_passed=True,ready_for_supervised_trial=True,physical_drive_validated=False)
    (root/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    return dict(path=str(root),goal_frame=conditions['goal_frame'],goal_distance_m=conditions['goal_distance_m'],look_execution=conditions['look_execution'],offline_checks_passed=True,physical_drive_validated=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial-dir',type=Path,required=True)
    p.add_argument('--results',type=Path,default=REPO/'.local-data/jetson-goal-campaign-audit-20260912/results.json')
    a=p.parse_args();print(json.dumps(validate(a.trial_dir,a.results),indent=2))


if __name__=='__main__':main()
