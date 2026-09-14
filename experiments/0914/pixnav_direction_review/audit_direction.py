"""Join raw policy actions to executed motion and recorded goal/pose evidence."""
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
REVIEW = OUT.parent / 'night_log_review'
CAMPAIGN = Path('/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-recaptured5-20260914')
ACTIONS = ['stop', 'move_forward', 'turn_left', 'turn_right', 'look_up', 'look_down']


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    with gzip.open(REVIEW / 'plot_data.json.gz', 'rt') as f:
        runs = json.load(f)
    report = {'scope': 'Five actual Direct trials to goal 4; no Direct trial to goal 5 in this campaign.',
              'measured_pose_is_odometry_not_external_ground_truth': True,
              'operator_conditions': 'Only 4-002 has confirmed no intervention/contact/obstruction. Others remain unconfirmed.',
              'runs': [], 'input_sha256': {}}
    commands = []
    for name in ['direct_goal-goal-4-%03d' % n for n in range(2, 7)]:
        root = REVIEW / 'episodes/recaptured5' / name
        policy = json.loads((root / 'policy_outputs.json').read_text())
        events = json.loads((root / 'motion_events.json').read_text())
        with gzip.open(root / 'evidence/full5m/policy-trace/pixnav_apply_trace.jsonl.gz', 'rt') as f:
            apply = [json.loads(line) for line in f if line.strip()]
        first = next(x for x in apply if x['event'] == 'apply_report')
        r = runs['recaptured5/' + name]
        goal = r['goal']
        ctrl = r['ctrl']
        terminal = [e for e in events if e['event'] == 'terminal']
        for e in terminal:
            step = int(e['command_id'].rsplit('-', 1)[-1])
            raw = policy[step]['action']
            kind = 'rotate' if raw in (2, 3) else 'translate'
            expected = math.pi/6 if raw == 2 else -math.pi/6 if raw == 3 else .2 if raw in (4,5) else .25
            assert raw != 0 and e['kind'] == kind
            assert abs(e['target'] - expected) < 1e-6
            assert e['status'] == 'OK'
            assert e['target'] * e['measured_progress'] > 0
            start = e['log_unix_s'] - e['elapsed_s']
            before = [c for c in ctrl if datetime.fromisoformat(c['utc']).timestamp() <= start]
            pose = before[-1]
            bearing = wrap(math.atan2(goal[1]-pose['y_m'], goal[0]-pose['x_m']) - pose['yaw_rad'])
            commands.append({'run': name, 'step_index': step, 'raw_action': raw,
                             'action_name': ACTIONS[raw], 'kind': kind,
                             'target': e['target'], 'measured_progress': e['measured_progress'],
                             'measured_turn_deg': math.degrees(e['measured_progress']) if kind == 'rotate' else None,
                             'elapsed_s': e['elapsed_s'], 'goal_bearing_before_action_deg': math.degrees(bearing),
                             'pose_receipt_to_action_start_s': start-datetime.fromisoformat(pose['utc']).timestamp(),
                             'controller_status': e['status'], 'direction_mapping_matches': True})
        assert len(terminal) == len(policy)-1
        assert policy[-1]['action'] == 0 and policy[-1]['stop_source'] == 'model'
        assert first['selected_view_commanded_preturn_yaw_deg'] == 0
        last = ctrl[-1]
        row = {'run': name, 'policy_outputs': len(policy), 'executed_actions': len(terminal),
               'turn_count': sum(e['kind'] == 'rotate' for e in terminal),
               'preturn_deg': first['selected_view_commanded_preturn_yaw_deg'],
               'initial_goal_pixel': first['original_selected_image_point'],
               'goal_xy_m': goal, 'initial_goal_bearing_deg': math.degrees(math.atan2(goal[1],goal[0])),
               'final_position_robot_origin_m': [last['x_m'], last['y_m']],
               'final_yaw_robot_origin_deg': math.degrees(last['yaw_rad']),
               'final_goal_distance_m': r['metric']['final_goal_distance_m'],
               'minimum_recorded_goal_distance_m': min(p['distance_m'] for p in r['pg']),
               'raw_action_sequence': [p['action'] for p in policy]}
        report['runs'].append(row)
        for filename in ('policy_outputs.json', 'motion_events.json', 'control_pose.csv'):
            report['input_sha256'][str((root/filename).relative_to(OUT.parent))] = sha(root/filename)
    report['executed_action_count'] = len(commands)
    report['turn_count'] = sum(x['kind'] == 'rotate' for x in commands)
    report['direction_mapping_mismatches'] = 0
    report['commands'] = commands
    (OUT / 'direction_results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    with (OUT / 'actions.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=list(commands[0]))
        w.writeheader(); w.writerows(commands)

    # A compact figure shows what was actually given to the policy and where it went.
    name = 'direct_goal-goal-4-006'
    r = runs['recaptured5/' + name]
    source = next((CAMPAIGN/'episodes'/name/'full5m/policy-inputs').glob('*/session_0001_reset.npz'))
    (OUT/'selected_inputs').mkdir(exist_ok=True)
    for p in (source, source.with_suffix('.json'), source.parent/'session_0001_step_0005.npz', source.parent/'session_0001_step_0005.json'):
        shutil.copy2(p, OUT/'selected_inputs'/p.name)
    with np.load(source) as z:
        initial = z['policy_goal_image'][0]
        mask = z['policy_goal_mask'][0,:,:,0]
    with np.load(source.parent/'session_0001_step_0005.npz') as z:
        obs = z['observation'][:,:,::-1]
    meta = json.loads((source.parent/'session_0001_step_0005.json').read_text())
    fig, axs = plt.subplots(1,3,figsize=(13,4.4), gridspec_kw={'width_ratios':[1,1.15,1.1]})
    ys,xs = np.where(mask>0)
    axs[0].imshow(initial);axs[0].plot(xs.mean(),ys.mean(),'o',mfc='none',mec='red',mew=2,ms=10)
    axs[0].set_title('Actual policy goal image\nRed: fixed initial target mask')
    axs[1].imshow(obs)
    v,u = meta['goal_prediction']
    axs[1].plot(u*obs.shape[1],v*obs.shape[0],'o',mfc='none',mec='yellow',mew=2,ms=10)
    axs[1].set_title('Recorded frame before LEFT choice\nYellow: model prediction, not measured goal')
    for ax in axs[:2]:ax.axis('off')
    xy=r['ctrl'];goal=r['goal']
    axs[2].plot([p['y_m'] for p in xy],[p['x_m'] for p in xy],c='tab:blue',lw=2,label='Recorded path')
    axs[2].plot(goal[1],goal[0],'r*',ms=14,label='Goal 4')
    axs[2].plot(0,0,'ko',label='Start')
    axs[2].add_patch(plt.Circle((goal[1],goal[0]),1,fill=False,color='red',ls='--'))
    axs[2].set_xlim(4,-6);axs[2].set_ylim(-.5,12);axs[2].set_aspect('equal')
    axs[2].set_xlabel('Lateral position: left (+) / right (-), m')
    axs[2].set_ylabel('Forward position (m)')
    axs[2].set_title('Odometry estimate\nActual turn: LEFT 32.47 degrees')
    axs[2].legend(fontsize=8);axs[2].grid(alpha=.2)
    fig.suptitle('Fifth actual Direct trial (goal 4, run 006): goal on right; policy chose left',fontsize=11)
    fig.tight_layout(rect=(0,0,1,.92))
    for extension in ('png','pdf'):
        fig.savefig(OUT/('last_trial_direction.'+extension),dpi=170,bbox_inches='tight')
    plt.close(fig)
    print(json.dumps({'actions':len(commands),'turns':report['turn_count'],'direction_mismatches':0,
                      'last_trial_turn':next(x for x in commands if x['run']==name and x['kind']=='rotate')},indent=2))


if __name__ == '__main__':
    main()
