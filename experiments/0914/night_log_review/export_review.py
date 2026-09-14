#!/usr/bin/env python3
"""Export recorded 13/14 September episodes; no ROS, network or robot commands.

All coordinates are estimates. Original episode/archive files are read only.
Run from any directory. Requires Python 3, numpy and matplotlib.
"""
import csv
import gzip
import hashlib
import importlib.util
import json
import math
import re
import shutil
from collections import Counter
from datetime import datetime, timezone, timedelta
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
ROOT = Path('/home/unitree/s2e-vlm-async-framework-minimal')
DATA = ROOT / '.local-data'
CAMPAIGNS = {
    'original5': 'recording-five-goals-20260913',
    'long5': 'recording-five-goals-long5-20260913',
    'recaptured5': 'recording-five-goals-recaptured5-20260914',
}
KST = timezone(timedelta(hours=9))
ACTION = {0: 'stop', 1: 'forward', 2: 'left', 3: 'right', 4: 'look_up', 5: 'look_down'}


def read_json(p, default=None):
    return json.loads(p.read_text()) if p.exists() else default


def rows(p):
    if not p.exists():
        return []
    with p.open() as f:
        return [json.loads(s) for s in f if s.strip()]


def write_json(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def write_csv(p, values, fields=None):
    p.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = list(dict.fromkeys(k for row in values for k in row))
    with p.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(values)


def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def copy_evidence(src, dest, inventory, compress=False):
    if not src.exists():
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    if compress:
        dest = dest.with_name(dest.name + '.gz')
        with src.open('rb') as fi, dest.open('wb') as fo:
            with gzip.GzipFile(filename='', fileobj=fo, mode='wb', mtime=0, compresslevel=6) as gz:
                shutil.copyfileobj(fi, gz)
    else:
        shutil.copyfile(src, dest)
    inventory.append(dict(source=str(src), exported=str(dest.relative_to(OUT)),
                          source_bytes=src.stat().st_size, source_sha256=digest(src),
                          exported_bytes=dest.stat().st_size, exported_sha256=digest(dest),
                          encoding='gzip' if compress else 'identity'))


def unix(value):
    return datetime.fromisoformat(value.replace('Z', '+00:00')).timestamp()


def stamp(value):
    return value['sec'] + value['nanosec'] * 1e-9


def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))


def local_pose(event, origin, start):
    d = event['data']
    pose = d['pose']['pose'] if event['event'] == 'odom' else d['pose']
    p, q = pose['position'], pose['orientation']
    if event['event'] == 'odom':
        dx, dy = p['x']-origin[0], p['y']-origin[1]
        c, s = math.cos(origin[2]), math.sin(origin[2])
        x, y, th = c*dx+s*dy, -s*dx+c*dy, yaw(q)-origin[2]
    else:
        x, y, th = p['x'], p['y'], yaw(q)
    return dict(t_s=event['elapsed_s']-start, utc=event['utc'],
                source_stamp_s=stamp(d['header']['stamp']),
                receipt_age_s=unix(event['utc'])-stamp(d['header']['stamp']),
                x_m=x, y_m=y, z_m=p.get('z'), yaw_rad=math.atan2(math.sin(th), math.cos(th)),
                source=event['event'], frame='episode_start' if event['event']=='odom' else 'robot_origin', raw_frame=d['header']['frame_id'])


def motion_rows(path):
    out = []
    if not path.exists():
        return out
    for line in path.open(errors='replace'):
        if 'robot_motion_event ' not in line:
            continue
        m = re.search(r'\[(\d{10}\.\d+)\]', line)
        try:
            value = json.loads(line.split('robot_motion_event ', 1)[1])
        except ValueError:
            continue
        value['log_unix_s'] = float(m.group(1)) if m else None
        out.append(value)
    return out


def figure_episode(dest, key, poses, pointgoals, goal, metric):
    if not poses:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.3))
    x, y = [p['x_m'] for p in poses], [p['y_m'] for p in poses]
    axes[0].plot(x, y, lw=1.4, label='Odometry estimate')
    axes[0].plot(0, 0, 'ko', label='Episode start')
    axes[0].plot(x[-1], y[-1], 's', color='tab:orange', label='Final recorded pose')
    if goal:
        axes[0].plot(*goal, '*', ms=13, color='tab:red', label='Saved point goal')
        axes[0].add_patch(plt.Circle(goal, 1, color='tab:red', fill=False, ls='--'))
    axes[0].axis('equal')
    axes[0].set(xlabel='Forward from start (m)', ylabel='Left from start (m)')
    axes[0].legend(fontsize=7)
    if pointgoals:
        axes[1].plot([p['t_s'] for p in pointgoals], [p['distance_m'] for p in pointgoals])
    axes[1].axhline(1, ls='--', color='tab:red', label='Configured 1 m arrival radius')
    axes[1].set(xlabel='Time since goal publication (s)', ylabel='Estimated goal distance (m)')
    axes[1].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=.25)
    warning = '\nINCLUDES OPERATOR RELOCATION; NOT AUTONOMOUS PATH LENGTH' if metric.get('operator_intervention_reviewed') else ''
    fig.suptitle(key + '\n' + str(metric['result_reason']) + warning, fontsize=10)
    fig.text(.5, .015, 'Fixed-start odometry; no ground-truth or global-map registration claim.', ha='center', fontsize=8)
    fig.tight_layout(rect=(0, .035, 1, .87 if warning else .91))
    fig.savefig(dest/'trajectory.png', dpi=150)
    fig.savefig(dest/'trajectory.pdf')
    plt.close(fig)


def export():
    spec = importlib.util.spec_from_file_location('episode_archive', ROOT/'scripts/robot_episode_archive.py')
    archive = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(archive)
    inventory, metrics, all_data = [], [], {}
    for short, base in CAMPAIGNS.items():
        campaign = DATA/base
        run_names = {p.parents[2].name for p in (campaign/'episodes').glob('*/full5m/runtime/result.json')}
        run_names |= {p.parent.name for p in (campaign/'archives').glob('*/episode.json')}
        for name in sorted(run_names):
            run, dest = campaign/'episodes'/name, OUT/'episodes'/short/name
            dest.mkdir(parents=True, exist_ok=True)
            result = read_json(run/'full5m/runtime/result.json', {})
            manifest = read_json(run/'manifest.json', {})
            events = rows(run/'full5m/runtime/events.jsonl')
            starts = [e for e in events if e['event'] == 'goal_published']
            stops = [e for e in events if e['event'] == 'direct_stop']
            derived, _ = archive.analyze(run)
            write_json(dest/'episode.recomputed.json', derived)
            archived = campaign/'archives'/name
            for q in ['episode.json', 'raw-manifest.json', 'odom-trajectory.csv']:
                copy_evidence(archived/q, dest/'original_archive'/q, inventory)
            for q in ['manifest.json', 'goal-plan.json', 'campaign-binding.json', 'recording-integrity.json',
                      'field-report.json', 'operator-invocation.json', 'camera-contract.json',
                      'policy-profile.yaml', 'compose.trial.yaml']:
                copy_evidence(run/q, dest/'evidence'/q, inventory)
            selected = ['navigation.log', 'pixnav.log', 'sidecar.log', 'native-recording.log',
                        'full5m/runtime/events.jsonl', 'full5m/runtime/result.json', 'full5m/runtime/goal.json',
                        'full5m/body-recording/events.jsonl', 'full5m/camera-timing/events.jsonl']
            selected += [str(p.relative_to(run)) for p in (run/'full5m/policy-trace').glob('*.json*')]
            for q in sorted(set(selected)):
                copy_evidence(run/q, dest/'evidence'/q, inventory, compress=q.endswith(('.log', '.jsonl')))
            policy = []
            for p in sorted((run/'full5m/policy-inputs').glob('*/session_*_step_*.json')):
                d = read_json(p)
                d['session_directory'] = p.parent.name
                policy.append(d)
            write_json(dest/'policy_outputs.json', policy)
            ctrl, odom, pg, status_changes, calls = [], [], [], [], []
            start = starts[0]['elapsed_s'] if starts else 0
            end = stops[0]['elapsed_s'] if stops else (events[-1]['elapsed_s'] if events else start)
            origin = [starts[0]['data'][k] for k in ['origin_x', 'origin_y', 'heading_rad']] if starts else [0, 0, 0]
            goal = starts[0]['data'].get('fixed_start_goal_xy') if starts else None
            goal_xy = [goal['x'], goal['y']] if goal else None
            previous = {}
            for e in events:
                if not start <= e['elapsed_s'] <= end:
                    continue
                if e['event'] == 'odom':
                    odom.append(local_pose(e, origin, start))
                elif e['event'] == 'control_pose':
                    ctrl.append(local_pose(e, origin, start))
                elif e['event'] == 'pointgoal':
                    pg.append(dict(t_s=e['elapsed_s']-start, utc=e['utc'], distance_m=e['data']['distance_m'],
                                   bearing_rad=e['data']['bearing_rad']))
                elif e['event'] == 'status':
                    d=e['data']; signature=(d['state'],d['active_mode'],d['is_healthy'],d['error_code'])
                    if previous.get(d['node_name']) != signature:
                        status_changes.append(dict(t_s=e['elapsed_s']-start,utc=e['utc'],node=d['node_name'],
                                                   state=d['state'],mode=d['active_mode'],healthy=d['is_healthy'],error=d['error_code']))
                        previous[d['node_name']] = signature
            start_unix = unix(starts[0]['utc']) if starts else None
            for d in rows(run/'full5m/policy-trace/vlm_calls.jsonl'):
                finish = unix(d['created_at_utc'])
                latency = d.get('latency_s')
                calls.append(dict(finished_utc=d['created_at_utc'],finished_kst=datetime.fromtimestamp(finish,KST).isoformat(),
                                  approximate_finish_t_s=finish-start_unix if start_unix else None,
                                  approximate_begin_t_s=finish-latency-start_unix if start_unix and latency else None,
                                  call_type=d.get('call_type'),latency_s=latency,model=d.get('model'),
                                  error_code=(d.get('error') or {}).get('error_code'),
                                  error_message=(d.get('error') or {}).get('message'),
                                  response_headers_wait_s=d.get('response_headers_wait_s')))
            write_csv(dest/'trajectory.csv', odom)
            write_csv(dest/'control_pose.csv', ctrl)
            write_csv(dest/'goal_distance.csv', pg)
            write_csv(dest/'status_transitions.csv', status_changes)
            write_csv(dest/'vlm_requests.csv', calls)
            motion=motion_rows(run/'navigation.log')
            write_json(dest/'motion_events.json', motion)
            terminal_policy=policy[-1] if policy else {}
            condition=manifest.get('runtime_conditions',{})
            integ=read_json(run/'recording-integrity.json',{})
            m=dict(campaign=short,run=name,started=bool(starts),
                   start_kst=datetime.fromtimestamp(start_unix,KST).isoformat() if start_unix else None,
                   navigation=condition.get('navigation'),goal=condition.get('goal_label'),
                   result_reason=result.get('reason','INCOMPLETE'),
                   odom_radius_reached=int(result.get('reason')=='GOAL_DISTANCE_REACHED') if starts else None,
                   archive_success=derived.get('success'),paper_success=None,paper_spl=None,
                   duration_s=derived.get('elapsed_to_stop_s'),final_goal_distance_m=result.get('goal_distance_m'),
                   minimum_goal_distance_m=derived.get('minimum_recorded_goal_distance_m'),
                   odom_path_10hz_m=(derived.get('path_length') or {}).get('10hz_m'),
                   odom_path_5hz_m=(derived.get('path_length') or {}).get('5hz_m'),
                   odom_path_20hz_m=(derived.get('path_length') or {}).get('20hz_m'),
                   pose_samples=len(ctrl),odom_samples=len(odom),
                   forward_speed_mps=condition.get('forward_speed_mps'),turn_speed_radps=condition.get('turn_speed_radps'),
                   look_execution=condition.get('look_execution'),max_duration_s=condition.get('max_duration_s'),
                   vlm_calls=len(calls),vlm_errors=sum(bool(d['error_code']) for d in calls),
                   policy_outputs=len(policy),raw_action_counts=json.dumps(dict(Counter(ACTION.get(d['action'],str(d['action'])) for d in policy))),
                   policy_final_action=ACTION.get(terminal_policy.get('action')),policy_stop_source=terminal_policy.get('stop_source'),
                   recording_integrity_passed=integ.get('passed'),recording_issues='; '.join(integ.get('issues',[])),
                   field_report_status=(derived.get('field_report') or {}).get('status'),
                   navigation_image=manifest.get('images',{}).get('escape'),
                   checkpoint_sha256=manifest.get('checkpoint',{}).get('sha256'),
                   evaluation_map_sha256=manifest.get('evaluation_map_sha256'),
                   metric_use='descriptive_only_pending_field_and_reference_review')
            if not starts:
                m['metric_use']='prestart_excluded_from_driving_denominator'
            if short=='recaptured5' and name=='full-goal-4-003':
                m['metric_use']='hold_path_and_success_pending_operator_movement_review'
            metrics.append(m)
            key=short+'/'+name
            all_data[key]=dict(metric=m,odom=odom,ctrl=ctrl,pg=pg,goal=goal_xy,calls=calls,motion=motion,start_unix=start_unix)
            figure_episode(dest,key,odom,pg,goal_xy,m)
            print('EXPORTED',key,len(odom),'poses',flush=True)
    write_csv(OUT/'episodes_summary.csv',metrics)
    write_json(OUT/'episodes_summary.json',metrics)
    write_json(OUT/'evidence_inventory.json',inventory)
    with (OUT/'plot_data.json.gz').open('wb') as f:
        with gzip.GzipFile(filename='',fileobj=f,mode='wb',mtime=0,compresslevel=6) as gz:
            gz.write(json.dumps(all_data,ensure_ascii=False,allow_nan=False).encode())
    copy_evidence(ROOT/'scripts/robot_episode_archive.py',OUT/'provenance/robot_episode_archive.py',inventory)
    write_json(OUT/'evidence_inventory.json',inventory)
    return all_data


if __name__ == '__main__':
    export()
