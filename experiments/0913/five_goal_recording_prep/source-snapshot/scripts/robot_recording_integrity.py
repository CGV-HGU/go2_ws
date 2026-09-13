#!/usr/bin/env python3
"""Check recorded files only. Does not launch ROS or issue motion commands."""
import argparse
import collections
import datetime
import json
from pathlib import Path
import time
import yaml

NATIVE_TOPICS = ('/robot_nav/sensors/front_camera/image_raw',
                 '/robot_nav/sensors/front_camera/camera_info',
                 '/s2e/robot/sensors/cloud','/s2e/robot/sensors/odom','/s2e/robot/sensors/imu')


def rows(path, issues, live=False):
    if not path.exists():
        issues.append('missing: '+str(path)); return []
    lines=path.read_text().splitlines(); values=[]
    for i,line in enumerate(lines):
        try: values.append(json.loads(line))
        except json.JSONDecodeError:
            if not (live and i==len(lines)-1): issues.append('invalid JSON: '+str(path)+':'+str(i+1))
    return values


def check(run, *, live=False, now=None):
    run=Path(run); now=time.time() if now is None else now
    issues=[]; streams={}; counts={}
    for label,relative in [('body','full5m/body-recording/events.jsonl'),
                           ('camera','full5m/camera-timing/events.jsonl'),
                           ('observation','full5m/observation-recording/events.jsonl')]:
        values=rows(run/relative,issues,live); streams[label]=values
        counts[label]=dict(collections.Counter(v.get('kind',v.get('event')) for v in values))
    for label,event,max_age in [('body','low',2.),('body','sport',2.),
                                ('camera','raw_image_header',2.),('camera','time_sync_status',5.),
                                ('observation','image',2.)]:
        selected=[v for v in streams[label] if v.get('kind',v.get('event'))==event]
        if not selected: issues.append('no recorded '+label+':'+event)
        elif live and not -.1 <= now-selected[-1].get('wall_ns',0)/1e9 <= max_age:
            issues.append('stale recorded '+label+':'+event)
    images=[v for v in streams['observation'] if v.get('event')=='image']
    if images and not (run/'full5m/observation-recording'/images[-1]['data']['path']).is_file():
        issues.append('observation image bytes missing')
    native_counts={}; started=None; outcome=None
    if live:
        log=(run/'native-recording.log').read_text() if (run/'native-recording.log').exists() else ''
        for topic in NATIVE_TOPICS:
            if "Subscribed to topic '"+topic+"'" not in log:
                issues.append('native recorder not subscribed: '+topic)
    else:
        events=rows(run/'full5m/runtime/events.jsonl',issues)
        started=any(v.get('event')=='goal_published' for v in events)
        for name in ('manifest.json','camera-contract.json','navigation.log','sidecar.log','pixnav.log','full5m/runtime/result.json'):
            if not (run/name).is_file(): issues.append('missing: '+name)
        result=run/'full5m/runtime/result.json'
        if result.exists(): outcome=json.loads(result.read_text()).get('reason')
        if started:
            for event in ('odom','control_pose','pointgoal','task','direct_stop'):
                if not any(v.get('event')==event for v in events): issues.append('no operator event: '+event)
            if not (run/'full5m/runtime/goal.json').exists(): issues.append('missing dispatched goal')
            bounds=[datetime.datetime.fromisoformat(v['utc']).timestamp() for v in events
                    if v.get('event') in ('goal_published','direct_stop') and v.get('utc')]
            if len(bounds)<2: issues.append('goal-to-stop wall time bounds missing')
            else:
                begin,end=min(bounds),max(bounds)
                for label,event in [('body','low'),('body','sport'),('camera','raw_image_header'),
                                    ('camera','time_sync_status'),('observation','image')]:
                    times=sorted(v['wall_ns']/1e9 for v in streams[label]
                                 if v.get('kind',v.get('event'))==event and 'wall_ns' in v)
                    inside=[t for t in times if begin<=t<=end]
                    if not times or times[0]>begin+1. or times[-1]<end-1.:
                        issues.append('recording does not span episode: '+label+':'+event)
                    if max((b-a for a,b in zip([begin]+inside,inside+[end])),default=0)>2.:
                        issues.append('recording gap over 2s: '+label+':'+event)
        commands=sum(v.get('event')=='command' for v in events)
        steps=list((run/'full5m/policy-inputs').glob('**/*step_*.json'))
        if commands and not steps: issues.append('policy command without saved policy input')
        manifest=run/'manifest.json'
        mode=json.loads(manifest.read_text()).get('runtime_conditions',{}).get('navigation') if manifest.exists() else None
        if commands and mode=='full':
            for name in ('vlm_calls.jsonl','async_decision_trace.jsonl','sweep_scheduler_trace.jsonl'):
                p=run/'full5m/policy-trace'/name
                if not p.exists() or p.stat().st_size==0: issues.append('Full policy trace missing: '+name)
        if manifest.exists() and json.loads(manifest.read_text()).get('runtime_conditions',{}).get('record_model_inputs'):
            path=run/'full5m/policy-trace/vlm_calls.jsonl'
            calls=rows(path,issues) if path.exists() else []
            if mode=='full' and commands and not calls: issues.append('VLM call evidence missing')
            if any('request' not in call for call in calls): issues.append('VLM request payload missing')
        for p in steps:
            if not p.with_suffix('.npz').exists(): issues.append('missing policy tensor: '+str(p))
        counts['operator']=dict(collections.Counter(v.get('event') for v in events))
        counts['policy_steps']=len(steps)
        metadata=run/'native-inputs/metadata.yaml'
        if metadata.exists():
            info=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
            native_counts={v['topic_metadata']['name']:v['message_count'] for v in info['topics_with_message_count']}
            for topic in NATIVE_TOPICS:
                if native_counts.get(topic,0)==0: issues.append('native topic empty: '+topic)
            for path in info.get('relative_file_paths',[]):
                if not (metadata.parent/path).is_file(): issues.append('missing native bag segment: '+path)
            begin=info['starting_time']['nanoseconds_since_epoch']/1e9
            end=begin+info['duration']['nanoseconds']/1e9
            bounds=[datetime.datetime.fromisoformat(v['utc']).timestamp() for v in events
                    if v.get('event') in ('goal_published','direct_stop') and v.get('utc')]
            if bounds and (begin>min(bounds)+1. or end<max(bounds)-1.):
                issues.append('native bag does not span goal-to-stop interval')
        else: issues.append('native bag not finalized: metadata.yaml missing')
    return dict(schema='escape-recording-integrity-v1',checked_unix=now,live_preflight=live,
                started=started,outcome=outcome,passed=not issues,issues=issues,
                stream_counts=counts,native_topic_counts=native_counts,
                note='File/subscription/receipt checks; not proof of zero transport loss or physical success. Missing artifacts are preserved as incomplete records.')


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--live',action='store_true');p.add_argument('--output',type=Path)
    a=p.parse_args();r=check(a.run_dir,live=a.live)
    text=json.dumps(r,indent=2,ensure_ascii=False)+'\n'
    if a.output:a.output.write_text(text)
    print(text,end='');return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
