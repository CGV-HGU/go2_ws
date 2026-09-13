"""Read-only analysis of 7 physical trials; never imports ROS command modules."""
import bisect, collections, json, math, statistics
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
F=ROOT/'.local-data/fixed-start-goals-20260913'
B=ROOT/'.local-data/old-map-fixedstart-goals-20260913'
def rows(p):
 if p.exists():
  with p.open() as f:
   for l in f:
    if l.strip():yield json.loads(l)
def stamp(d):
 s=d['header']['stamp'];return s['sec']+s['nanosec']/1e9
def summary(v):
 if not v:return {'n':0}
 s=sorted(v);return dict(n=len(s),min=s[0],median=statistics.median(s),p95=s[min(len(s)-1,int(len(s)*.95))],max=s[-1])
results=[]
for trial in [F/'prepared'/f'goal-{g}-{n:03d}' for g in (1,2) for n in (1,2,3)]+[B/'prepared/old-map-goal-2-002']:
 full=trial/'full5m'; ev=list(rows(full/'runtime/events.jsonl'))
 goal=json.loads((full/'runtime/goal.json').read_text())
 published=next(e for e in ev if e['event']=='goal_published');t0=published['elapsed_s']
 t1=next(e['elapsed_s'] for e in ev if e['event']=='direct_stop')
 live=[e for e in ev if t0<=e['elapsed_s']<=t1]
 odom=sorted((stamp(e['data']),e['data']['pose']['pose']['position']) for e in ev if e['event']=='odom')
 times=[o[0] for o in odom];dist_errors=[];bearing_errors=[]
 odomyaw={stamp(e['data']):e['data']['pose']['pose']['orientation'] for e in ev if e['event']=='odom'}
 for e in live:
  if e['event']!='pointgoal':continue
  d=e['data'];ts=stamp(d);i=bisect.bisect_right(times,ts)-1
  if i<0 or ts-times[i]>.1:continue
  p=odom[i][1];dx=goal['goal_x']-p['x'];dy=goal['goal_y']-p['y']
  dist_errors.append(abs(math.hypot(dx,dy)-d['distance_m']))
  q=odomyaw[times[i]];yaw=math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
  err=math.atan2(dy,dx)-yaw-d['bearing_rad'];bearing_errors.append(abs(math.degrees(math.atan2(math.sin(err),math.cos(err)))))
 commands=[e for e in live if e['event']=='command'];acks={e['data']['command_id']:e for e in live if e['event']=='result'}
 action_records=[]
 for i,c in enumerate(commands):
  d=c['data'];a=acks.get(d['command_id']);row=dict(action=d['action'],distance_m=d['forward_distance_m'],start_s=c['elapsed_s']-t0,command_id=d['command_id'])
  if a:
   row.update(duration_s=a['elapsed_s']-c['elapsed_s'],end_s=a['elapsed_s']-t0,status=a['data']['status'])
   if i+1<len(commands):row['next_command_gap_s']=commands[i+1]['elapsed_s']-a['elapsed_s']
  action_records.append(row)
 handoff=list(rows(full/'policy-trace/pixnav_handoff_trace.jsonl'));counts=collections.Counter(x['event'] for x in handoff)
 look=[x for x in handoff if x['event']=='look_forward_substitution']
 motion=[]
 for l in (trial/'navigation.log').read_text(errors='replace').splitlines():
  if 'robot_motion_event ' in l:
   try:motion.append(json.loads(l.split('robot_motion_event ',1)[1]))
   except ValueError:pass
 terminals=[e for e in motion if e.get('event')=='terminal']; rotations=[]; current=[]
 for e in motion:
  if e.get('event')=='rotate_command':current.append(e)
  if e.get('event')=='terminal' and current:
   changes=[x for i,x in enumerate(current) if i==0 or x.get('phase')!=current[i-1].get('phase')]
   rotations.append(dict(target_deg=math.degrees(current[0]['target']),status=e.get('status'),elapsed_s=e.get('elapsed_s'),final_error_deg=math.degrees(e.get('progress', e.get('measured_progress',0))-current[0]['target']),phase_changes=len(changes)-1,terminal=e,transitions=changes));current=[]
 calls=collections.defaultdict(list)
 for c in rows(full/'policy-trace/vlm_calls.jsonl'):
  if isinstance(c.get('latency_s'),(int,float)):calls[c['call_type']].append(c['latency_s'])
 bind=next(e['data'] for e in ev if e['event']=='fixed_start_binding');s=bind['episode_start_in_odom'];v=bind['saved_goal']['point_goal_xy_m']
 gx=s[0]+math.cos(s[2])*v['x']-math.sin(s[2])*v['y'];gy=s[1]+math.sin(s[2])*v['x']+math.cos(s[2])*v['y']
 expected=math.hypot(gx-goal['goal_x'],gy-goal['goal_y'])
 r=dict(trial=trial.name,elapsed_s=t1-t0,goal_distance_m=goal['distance_m'],goal_label=goal['goal_label'],frame=goal['frame'],pose_method=goal['localization_method'],goal_binding_error_m=expected,source_odom_age_at_dispatch_s=goal['origin_age_s'],pointgoal_distance_error_vs_previous_odom_m=summary(dist_errors),pointgoal_bearing_error_vs_previous_odom_deg=summary(bearing_errors),commands=len(commands),ack_count=len(acks),action_status=dict(collections.Counter(x.get('status','missing') for x in action_records)),look_substitution_count=len(look),handoff_events=dict(counts),vlm_latency_s={k:summary(v) for k,v in calls.items()},forward_10cm_duration_s=summary([x['duration_s'] for x in action_records if x['action']=='move_forward' and x['distance_m']<.15 and 'duration_s'in x]),forward_25cm_duration_s=summary([x['duration_s'] for x in action_records if x['action']=='move_forward' and x['distance_m']>.15 and 'duration_s'in x]),short_gap_between_actions_s=summary([x['next_command_gap_s'] for x in action_records if 'next_command_gap_s'in x and x['next_command_gap_s']<1]),motion_terminals=len(terminals),rotation_count=len(rotations),rotation_statuses=dict(collections.Counter(str(x['status']) for x in rotations)),rotation_phase_changes=summary([x['phase_changes'] for x in rotations]),initial_no_pixel_action_s=action_records[0]['start_s'] if action_records else t1-t0,final_no_pixel_action_s=t1-t0-action_records[-1].get('end_s',action_records[-1]['start_s']) if action_records else t1-t0)
 (OUT/(trial.name+'-actions.json')).write_text(json.dumps(action_records,indent=2)+'\n')
 (OUT/(trial.name+'-rotations.json')).write_text(json.dumps(rotations,indent=2)+'\n');results.append(r)
coverage=[]
for folder in ['prepared-daylight-map-20260913','reuse-night-map-20260913-second-check']:
 for p in sorted((ROOT/'.local-data'/folder).glob('native-inputs*/metadata.yaml')):
  m=yaml.safe_load(p.read_text())['rosbag2_bagfile_information'];coverage.append(dict(bag=str(p.parent.relative_to(ROOT)),start_unix=m['starting_time']['nanoseconds_since_epoch']/1e9,duration_s=m['duration']['nanoseconds']/1e9,topics={x['topic_metadata']['name']:x['message_count'] for x in m['topics_with_message_count']}))
sensitivity=[dict(distance_m=d,heading_error_deg=a,goal_shift_m=2*d*math.sin(math.radians(a)/2)) for d in [3.1146127959,4.8407019721,10,12.5262235999] for a in [1,3,5]]
report=dict(scope='Recorded data; odometry not ground truth; no physical command transport',episodes=results,native_recording_coverage=coverage,heading_alignment_sensitivity=sensitivity,pointgoal_comparison_note='Nearest preceding raw odometry <= 100 ms old, so dynamic discrepancy includes sampling delay; no ICP transform used.')
(OUT/'recording-audit.json').write_text(json.dumps(report,indent=2)+'\n')
for r in results:print(json.dumps({k:r[k] for k in ['trial','goal_binding_error_m','commands','ack_count','look_substitution_count','pointgoal_distance_error_vs_previous_odom_m','rotation_statuses']},ensure_ascii=False))
