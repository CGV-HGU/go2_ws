#!/usr/bin/env python3
"""Analyze immutable episode exports. No motion, network, or runtime mutations."""
import collections
import csv
from datetime import datetime
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
SOURCE = OUT.parent / 'night_log_review'


def read_rows(p):
    if not p.exists():
        return []
    with gzip.open(p, 'rt') as f:
        return [json.loads(line) for line in f if line.strip()]


def unix(s):
    return datetime.fromisoformat(s.replace('Z', '+00:00')).timestamp()


def stats(xs):
    return dict(count=len(xs), median=float(np.median(xs)), p95=float(np.percentile(xs, 95)),
                minimum=float(min(xs)), maximum=float(max(xs))) if xs else dict(count=0)


def union(intervals):
    result = []
    for a, b in sorted(intervals):
        if b <= a:
            continue
        if result and a <= result[-1][1]:
            result[-1][1] = max(result[-1][1], b)
        else:
            result.append([a, b])
    return result


def measure(intervals):
    return sum(b-a for a, b in union(intervals))


def intersect(left, right):
    return measure([(max(a,c), min(b,d)) for a,b in union(left) for c,d in union(right) if max(a,c)<min(b,d)])


def dump(name, value):
    (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def csv_write(name, rows):
    with (OUT/name).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        w.writeheader()
        w.writerows(rows)


def analyze(entry):
    label = entry['campaign']+'/'+entry['run']
    root = SOURCE/'episodes'/label
    events_path = root/'evidence/full5m/runtime/events.jsonl.gz'
    events = read_rows(events_path)
    goals = [unix(e['utc']) for e in events if e['event']=='goal_published']
    stops = [unix(e['utc']) for e in events if e['event']=='direct_stop']
    if not goals or not stops:
        return None
    begin, end = goals[0], next(t for t in stops if t>=goals[0])
    duration = end-begin
    motion = json.loads((root/'motion_events.json').read_text())
    odom = []
    for e in events:
        if e['event']!='odom':
            continue
        v=e['data'];stamp=v['header']['stamp'];pos=v['pose']['pose']['position'];vel=v['twist']['twist']['linear']
        odom.append((stamp['sec']+stamp['nanosec']*1e-9, pos['x'], pos['y'], math.hypot(vel['x'],vel['y'])))
    odom = np.array(sorted(odom))
    command_events = {e['data']['command_id']: e for e in events if e['event']=='command'}
    result_events = {e['data']['command_id']: e for e in events if e['event']=='result'}
    calls=read_rows(root/'evidence/full5m/policy-trace/vlm_calls.jsonl.gz')
    call_intervals=[]
    for c in calls:
        if c.get('created_at_utc') and c.get('latency_s') is not None:
            stop=unix(c['created_at_utc']);start=stop-float(c['latency_s'])
            if max(begin,start)<min(end,stop):call_intervals.append([max(begin,start),min(end,stop)])
    macros=[];pending=[];intervals=[];translation_drive=[]
    for e in motion:
        if e['event']!='terminal':
            pending.append(e);continue
        finish=e['log_unix_s'];start=finish-e['elapsed_s']
        ticks=[c for c in pending if c['event']==e['kind']+'_command' and c['log_unix_s']>=start-.12]
        pending=[]
        if finish<begin or start>end:
            continue
        a,b=max(start,begin),min(finish,end)
        intervals.append([a,b])
        category=('look_forward_20cm' if abs(e['target']-.2)<1e-5 else
                  'look_forward_10cm' if abs(e['target']-.1)<1e-5 else 'forward') if e['kind']=='translate' else (
                  'policy_turn' if e['command_id'].startswith('pixnav') else 'observation_or_alignment_turn')
        active=rest=0.;moving_speeds=[];corrective_count=0;active_tick_count=0;rate_samples=[]
        near_peak=0.;first_zero=None;phases=[]
        for i,c in enumerate(ticks):
            t=c['elapsed_s'];nxt=ticks[i+1]['elapsed_s'] if i+1<len(ticks) else e['elapsed_s']
            # Do not infer sustained actuation through a long unobserved tick gap.
            dt=max(0,min(nxt-t,.15));drive=abs(c.get('linear_command_mps',0))>1e-6 if e['kind']=='translate' else abs(c.get('angular_command_radps',0))>1e-6
            active+=dt if drive else 0;rest+=0 if drive else dt
            phases.append((t,'drive' if drive else 'rest'))
            if not drive and first_zero is None:first_zero=t
            if e['kind']=='rotate' and drive:rate_samples.append(abs(c['measured_yaw_rate_radps']))
            if e['kind']=='translate' and drive:
                active_tick_count+=1
                corrective_count+=abs(c.get('angular_command_radps',0))>.01
                if c['linear_command_mps']>=.49:near_peak+=dt
                translation_drive.append([max(begin,start+t),min(end,start+t+dt)])
                stamp=c['odom_stamp_ns']*1e-9
                idx=int(np.searchsorted(odom[:,0],stamp))
                close=min([j for j in (idx-1,idx) if 0<=j<len(odom)],key=lambda j:abs(odom[j,0]-stamp))
                if abs(odom[close,0]-stamp)<=.075:moving_speeds.append(float(odom[close,3]))
        cmd=command_events.get(e['command_id']);res=result_events.get(e['command_id'])
        command_to_start=start-unix(cmd['utc']) if cmd else None
        result_delay=unix(res['utc'])-finish if res else None
        # Approximate wall start comes from a terminal log AFTER synchronous StopMove.
        row=dict(run=label,index=len(macros),kind=e['kind'],category=category,command_id=e['command_id'],
                 target=e['target'],progress=e['measured_progress'],status=e['status'],
                 start_unix=start,end_unix=finish,elapsed_s=e['elapsed_s'],
                 observed_nonzero_command_s=active,observed_zero_command_s=rest,
                 unclassified_macro_s=max(0,e['elapsed_s']-active-rest),
                 mean_progress_per_macro_s=abs(e['measured_progress'])/e['elapsed_s'],
                 first_zero_command_elapsed_s=first_zero,correction_restarts=sum(x[1]=='rest' and y[1]=='drive' for x,y in zip(phases,phases[1:])),
                 command_near_0p5_s=near_peak,translation_nonzero_ticks=active_tick_count,
                 translation_with_yaw_correction_ticks=corrective_count,
                 measured_moving_twist_median_mps=float(np.median(moving_speeds)) if moving_speeds else None,
                 measured_turn_rate_median_radps=float(np.median(rate_samples)) if rate_samples else None,
                 matched_moving_twist_samples=len(moving_speeds),
                 command_receipt_to_start_s=command_to_start,terminal_to_result_receipt_s=result_delay)
        macros.append(row)
    category_seconds=collections.defaultdict(float)
    for m in macros:category_seconds[m['category']]+=max(0,min(m['end_unix'],end)-max(m['start_unix'],begin))
    macro_union=measure(intervals)
    coverage_overlap=sum(b-a for a,b in intervals)-macro_union
    assert coverage_overlap<.5, (label,coverage_overlap)
    by_type=collections.defaultdict(list)
    for c in calls:
        if c.get('latency_s') is not None:by_type[c.get('call_type','unknown')].append(float(c['latency_s']))
    row=dict(run=label,result=entry['result_reason'],navigation=entry['navigation'],goal=entry['goal'],
             look_execution=entry['look_execution'],turn_speed_radps=entry['turn_speed_radps'],
             duration_s=duration,final_goal_distance_m=entry['final_goal_distance_m'],
             path_10hz_m=entry['odom_path_10hz_m'],path_per_episode_s=entry['odom_path_10hz_m']/duration,
             macro_count=len(macros),macro_covered_s=macro_union,other_time_s=max(0,duration-macro_union),
             macro_overlap_s=coverage_overlap,vlm_call_count=len(calls),vlm_error_count=sum(bool(c.get('error')) for c in calls),
             http_pending_union_s=measure(call_intervals),http_overlaps_macro_s=intersect(call_intervals,intervals),
             http_overlaps_nonzero_translation_command_s=intersect(call_intervals,translation_drive),
             http_pending_outside_macros_s=measure(call_intervals)-intersect(call_intervals,intervals),
             http_latency_median_s=stats([float(c['latency_s']) for c in calls if c.get('latency_s') is not None]).get('median'),
             navigation_http_median_s=stats(by_type['navigation']).get('median'),
             navigation_http_p95_s=stats(by_type['navigation']).get('p95'),
             translation_nonzero_command_s=sum(m['observed_nonzero_command_s'] for m in macros if m['kind']=='translate'),
             translation_zero_command_s=sum(m['observed_zero_command_s'] for m in macros if m['kind']=='translate'),
             rotation_nonzero_command_s=sum(m['observed_nonzero_command_s'] for m in macros if m['kind']=='rotate'),
             rotation_zero_command_s=sum(m['observed_zero_command_s'] for m in macros if m['kind']=='rotate'),
             initial_to_first_translate_s=min((m['start_unix']-begin for m in macros if m['kind']=='translate'),default=None),
             **{k+'_s':v for k,v in category_seconds.items()})
    return row, macros, events_path


def main():
    entries=json.loads((SOURCE/'episodes_summary.json').read_text())
    episodes=[];macros=[];hashes={}
    for entry in entries:
        if not entry['started']:continue
        value=analyze(entry)
        if value is None:continue
        row,details,src=value;episodes.append(row);macros.extend(details)
        for f in (src, src.parents[3]/'motion_events.json'):
            if f.exists():hashes[str(f.relative_to(OUT.parent))]=hashlib.sha256(f.read_bytes()).hexdigest()
    csv_write('episodes.csv',episodes);csv_write('macros.csv',macros)
    groups={}
    selected={e['run'] for e in episodes if e['look_execution']=='forward_0p2' and e['turn_speed_radps']==.8}
    for category in sorted({m['category'] for m in macros}):
        ms=[m for m in macros if m['run'] in selected and m['category']==category and m['status']=='OK']
        groups[category]=dict(elapsed_s=stats([m['elapsed_s'] for m in ms]),
          measured_progress_per_macro_s=stats([m['mean_progress_per_macro_s'] for m in ms]),
          moving_twist_mps=stats([m['measured_moving_twist_median_mps'] for m in ms if m['measured_moving_twist_median_mps'] is not None]),
          measured_turn_rate_radps=stats([m['measured_turn_rate_median_radps'] for m in ms if m['measured_turn_rate_median_radps'] is not None]),
          zero_command_s=stats([m['observed_zero_command_s'] for m in ms]),
          result_receipt_delay_s=stats([m['terminal_to_result_receipt_s'] for m in ms if m['terminal_to_result_receipt_s'] is not None]),
          command_receipt_to_start_s=stats([m['command_receipt_to_start_s'] for m in ms if m['command_receipt_to_start_s'] is not None]))
    dump('results.json',dict(episodes=episodes,matched_settings_macro_statistics=groups,
         selected_settings='look20cm, turn0.8rad/s; descriptive action timing, no efficacy claim',
         limitations=['Episode path speed is path length/time including waits and turns, not cruising speed.',
          'Odom twist nearest match <=75ms; external ground truth and exact controller-filtered speed unavailable.',
          'Nonzero/zero command spans use log monotonic intervals, cap extrapolation at150ms. Unclassified time retained.',
          'HTTP intervals inferred from completion UTC minus latency; overlapping calls unioned, clocks about1s precision.',
          'HTTP outside macros is coincident pending time, not causal proof of all wait time.',
          'recaptured5/full-goal-4-003 includes confirmed manual relocation; its path speed is not autonomous performance.',
          'Non-policy rotate includes observation and alignment, not all separately identified as full sweeps.'],
         source_sha256=hashes))
    print(json.dumps(dict(episodes=len(episodes),macros=len(macros),statistics=groups),indent=2))


if __name__=='__main__':main()
