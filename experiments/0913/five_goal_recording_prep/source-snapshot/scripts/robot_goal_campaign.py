#!/usr/bin/env python3
"""Prepare and archive five-goal campaigns. Never start ROS or publish a goal."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time
import uuid

from fixed_start_goals import make_plan, validate_plan
from prepare_robot_pointgoal_trial import prepare as prepare_trial
from validate_robot_pointgoal_trial import validate as validate_trial
from robot_map_session import prepare as prepare_map
from robot_episode_archive import archive
from robot_recording_integrity import check as check_recording

REPO=Path(__file__).resolve().parents[1]


def read(path):return json.loads(Path(path).read_text())
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n');tmp.replace(path)


def prepare_camera(root):
    """Freeze the separate camera publisher needed before mapping or driving."""
    rendered=json.loads(subprocess.check_output(['docker','compose','--env-file',str(REPO/'config/robot-full.env'),
        '-f',str(REPO/'compose.robot-minimal.yaml'),'-f',str(REPO/'compose.robot-full.yaml'),'config','--format','json'],text=True))
    camera=rendered['services']['camera']
    camera['image']=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',camera['image']],text=True).strip()
    camera['restart']='no';config=root/'camera-config';config.mkdir()
    for i,volume in enumerate(camera['volumes']):
        if volume['type']!='bind':raise ValueError('Unexpected camera volume')
        source=Path(volume['source']);target=config/(str(i)+'-'+source.name)
        if source.is_dir():shutil.copytree(source,target)
        else:shutil.copy2(source,target)
        volume['source']=str(target)
    write(root/'camera.compose.json',dict(name='escape-five-goal-camera',services={'camera':camera}))
    return dict(image=camera['image'],compose=str(root/'camera.compose.json'),
                sha256={str(p.relative_to(root)):digest(p) for p in [root/'camera.compose.json']+list(config.rglob('*')) if p.is_file()})


def prepare_campaign(root, validation):
    root=Path(root).resolve();validation=Path(validation).resolve();result=read(validation)
    if result.get('offline_checks_passed') is not True:
        raise ValueError('Candidate offline checks are incomplete')
    root.mkdir(parents=True,exist_ok=False)
    for name in ('goals','episodes','archives'): (root/name).mkdir()
    prepare_map(root/'mapping',result['navigation_image_id'])
    camera=prepare_camera(root)
    shutil.copy2(validation,root/'candidate-validation.json')
    value=dict(schema='escape-five-goal-campaign-v1',campaign_id=uuid.uuid4().hex,
               created_unix=time.time(),status='awaiting_mapping',required_goals=['1','2','3','4','5'],
               pose_method='fixed_start_odometry',same_marked_start_each_episode=True,
               validation=str(root/'candidate-validation.json'),navigation_image=result['navigation_image_id'],
               physical_drive_validated=False,free_disk_bytes_at_prepare=shutil.disk_usage(root).free,
               trials=[],primary_method='full',comparison_method='direct_goal',camera=camera)
    write(root/'campaign.json',value);return value


def bind_goals(root):
    root=Path(root).resolve();c=read(root/'campaign.json')
    if c['status']!='awaiting_mapping':raise ValueError('Campaign goal set is already sealed')
    m=read(root/'mapping/manifest.json')
    if m['status'] not in ('collecting_fixed_start_goals','stopped') or m.get('localization_mode')!='fixed_start_odometry':
        raise ValueError('Freeze this campaign map for fixed-start goal collection first')
    database=Path(m['source_database'])
    if database.resolve()!=root/'mapping/map.db' or digest(database)!=m['source_database_sha256']:
        raise ValueError('Campaign map does not match the frozen database')
    origin=read(root/'goals/origin.json')
    if origin['host_boot_id']!=m['host_boot_id'] or origin['captured_unix']<c['created_unix']:
        raise ValueError('Origin does not belong to this mapping/goal collection session')
    paths=sorted((root/'goals').glob('[1-5].json'))
    if {p.stem for p in paths}!=set(c['required_goals']):
        raise ValueError('Capture all five goals before sealing the episode set')
    plan=make_plan(origin,[read(p) for p in paths])
    for record in [origin]+plan['goals']:
        evidence=record.get('source_snapshot_file')
        if not evidence or Path(evidence).name!=evidence or digest(root/'goals'/evidence)!=record.get('source_snapshot_sha256'):
            raise ValueError('Missing/changed raw snapshot for a new campaign goal capture')
    for goal in plan['goals']:
        xy=goal['point_goal_xy_m']
        if xy['x']**2+xy['y']**2<=plan['success_radius_m']**2:
            raise ValueError('Goal '+goal['label']+' is already inside the start arrival radius')
    write(root/'goal-plan.json',plan)
    c.update(status='goals_sealed',goal_plan_sha256=digest(root/'goal-plan.json'),
             map_database=str(database),map_sha256=m['source_database_sha256'],
             origin_id=origin['origin_id'],icp_corrections_used=False)
    write(root/'campaign.json',c);return c


def add_trial(root,label,mode='full'):
    root=Path(root).resolve();c=read(root/'campaign.json');label=str(label)
    if c['status']!='goals_sealed':raise ValueError('Seal this campaign\'s five goals first')
    if mode not in ('full','direct_goal'):raise ValueError('Unknown navigation method')
    if digest(root/'goal-plan.json')!=c['goal_plan_sha256'] or digest(c['map_database'])!=c['map_sha256']:
        raise ValueError('Frozen map or goals changed')
    validate_plan(read(root/'goal-plan.json'),label)
    prefix=mode+'-goal-'+label+'-';attempt=1
    while (root/'episodes'/(prefix+str(attempt).zfill(3))).exists():attempt+=1
    trial_id=prefix+str(attempt).zfill(3);out=root/'episodes'/trial_id
    entry=dict(trial_id=trial_id,goal_label=label,method=mode,path=str(out),status='preparing')
    c['trials'].append(entry);write(root/'campaign.json',c)
    try:
        prepare_trial(out,fixed_start_plan=root/'goal-plan.json',goal_label=label,
                      navigation_mode=mode,navigation_image=c['navigation_image'])
        binding={key:c[key] for key in ('campaign_id','map_database','map_sha256','goal_plan_sha256','origin_id')}
        binding.update(trial_id=trial_id,goal_label=label,method=mode,pose_method='fixed_start_odometry')
        write(out/'campaign-binding.json',binding)
        write(out/'field-report.json',dict(status='awaiting_field_report',direct_intervention=None,
              contact=None,obstruction=None,physical_within_goal_radius_verified=None,
              physical_final_distance_m=None,measurement_method=None,operator_comment=None))
        m=read(out/'manifest.json');m.update(campaign_binding=binding,evaluation_map_sha256=c['map_sha256'])
        for name in ('campaign-binding.json','field-report.json'):m['sha256'][name]=digest(out/name)
        write(out/'manifest.json',m)
        validate_trial(out,c['validation'])
        entry['status']='prepared'
    except Exception as exc:
        entry.update(status='prepare_failed',error=str(exc));write(root/'campaign.json',c);raise
    write(root/'campaign.json',c);return entry


def archive_trial(root,trial_id):
    root=Path(root).resolve();c=read(root/'campaign.json')
    entries=[v for v in c['trials'] if v['trial_id']==trial_id]
    if len(entries)!=1:raise ValueError('Unknown campaign episode')
    entry=entries[0];run=Path(entry['path'])
    if entry['status']=='archived':raise ValueError('Episode already archived')
    # The wrapper writes this marker only after stopping all writers. An
    # interrupted process without it is retained for a separate recovery audit.
    if not (run/'RECORDERS_CLOSED').exists():raise ValueError('Finish stopping recorders before archiving')
    report=check_recording(run);write(run/'recording-integrity.json',report)
    out=root/'archives'/trial_id
    try:result=archive(run,out)
    except Exception as exc:
        entry.update(status='archive_failed',archive_error=str(exc));write(root/'campaign.json',c);raise
    entry.update(status='archived',archive=str(out),recording_complete=report['passed'],
                 started=result['started'],episode_id=result['episode_id'],result_reason=result['result_reason'])
    write(root/'campaign.json',c);return entry


def status(root):
    root=Path(root);c=read(root/'campaign.json')
    for entry in c['trials']:
        run=Path(entry['path'])
        if entry['status'] in ('archived','archive_failed'):continue
        result=run/'full5m/runtime/result.json'
        if result.exists():
            entry.update(status='awaiting_archive',result_reason=read(result).get('reason'))
        elif (run/'USED').exists():entry['status']='started_or_interrupted_before_result'
        if (run/'recording-integrity.json').exists():entry['recording_complete']=read(run/'recording-integrity.json')['passed']
    return c


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('prepare','seal','prepare-trial','archive','status'))
    p.add_argument('--root',type=Path,required=True);p.add_argument('--validation',type=Path)
    p.add_argument('--goal-label',choices=('1','2','3','4','5'))
    p.add_argument('--mode',choices=('full','direct_goal'),default='full');p.add_argument('--trial-id')
    a=p.parse_args()
    if a.action=='prepare':
        if not a.validation:p.error('Prepare requires --validation')
        result=prepare_campaign(a.root,a.validation)
    elif a.action=='seal':
        result=bind_goals(a.root)
        for label in result['required_goals']:add_trial(a.root,label,'full')
        result=read(a.root/'campaign.json')
    elif a.action=='prepare-trial':
        if not a.goal_label:p.error('Require --goal-label')
        result=add_trial(a.root,a.goal_label,a.mode)
    elif a.action=='archive':
        if not a.trial_id:p.error('Require --trial-id')
        result=archive_trial(a.root,a.trial_id)
    else:result=status(a.root)
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
