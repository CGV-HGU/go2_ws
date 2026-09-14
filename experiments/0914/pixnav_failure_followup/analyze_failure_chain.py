#!/usr/bin/env python3
"""Recompute failure-chain evidence from committed logs; no robot/ROS/model calls."""
import argparse
import csv
import datetime as dt
import gzip
import hashlib
import io
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=HERE)
args = parser.parse_args()
out = args.output
out.mkdir(parents=True, exist_ok=True)
provenance = {}


def read(path):
    data = path.read_bytes()
    relative = str(path.relative_to(ROOT))
    provenance[relative] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
    return gzip.decompress(data).decode() if path.suffix == '.gz' else data.decode()


def js(path):
    return json.loads(read(path))


def lines(path):
    return [json.loads(x) for x in read(path).splitlines() if x.strip()]


def save(name, value):
    (out / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def csvout(name, rows):
    with (out / name).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


night = ROOT / 'night_log_review'
base = night / 'episodes/original5/full-goal-2-006'
trace = base / 'evidence/full5m/policy-trace'
events = lines(base / 'evidence/full5m/runtime/events.jsonl.gz')
goal_event = next(e for e in events if e['event'] == 'goal_published')
goal_time = dt.datetime.fromisoformat(goal_event['utc']).timestamp()
stop_event = next(e for e in events if e['event'] == 'direct_stop')
distances = list(csv.DictReader(io.StringIO(read(base / 'goal_distance.csv'))))
closest = min(distances, key=lambda r: float(r['distance_m']))
policy = js(base / 'policy_outputs.json')
sessions = {}
for row in policy:
    sessions.setdefault(row['session_index'], []).append(row)
assert list(sessions) == list(range(1, 14))
recovery = lines(trace / 'recovery_plan_trace.jsonl.gz')
outcomes = [r for r in recovery if r['event'] == 'pixnav_navigation_outcome_recorded']
applies = {r['decision_id']: r for r in lines(trace / 'pixnav_apply_trace.jsonl.gz')
           if r['event'] == 'apply_report'}
async_rows = lines(trace / 'async_decision_trace.jsonl.gz')
calls = lines(trace / 'vlm_calls.jsonl.gz')
nav_calls = {r['decision_id']: r for r in calls if r.get('decision_id')}
assert len(outcomes) == len(sessions) == 13
chain = []
critical = {'d4e043e8fe1955234904e10c', 'a7055b5e54ec20cb7a314521',
            '467710f176ba0ef3b62fe4b6', '8cc79bd46b707aa64c8d476b'}
canonical_inputs = []
for index, outcome in enumerate(outcomes, 1):
    decision = outcome['decision_id']
    call = nav_calls[decision]
    text = next(c['text'] for m in call['request']['messages'] if m['role'] == 'user'
                for c in m['content'] if c['type'] == 'text')
    canonical = json.loads(text.split('VLM_INPUT_JSON:\n', 1)[1])
    context = canonical['memory']['goal_context']
    selected = next(c for c in canonical['pixel_candidates']['candidates']
                    if c['candidate_ref'] == call['selected_candidate_ref'])
    steps = sorted(sessions[index], key=lambda r: r['step_index'])
    assert [p['step_index'] for p in steps] == list(range(len(steps)))
    # Policy session files contain chronological session indices, not decision IDs.
    # Join against the 13 ordered completed decision outcomes and expose this method.
    apply = applies[decision]
    end_ns = apply['now_ns'] + outcome['event_monotonic_ns'] - apply['event_monotonic_ns']
    timing = next((r for r in async_rows if r.get('decision_id') == decision
                   and r['event'] == 'terminal_observation_timing_decision'), {})
    refresh = next((r for r in async_rows if r.get('decision_id') == decision
                    and r['event'] == 'pointnav_terminal_goal_bearing_refresh_queued'), {})
    chain.append({
        'session_index_chronological': index, 'decision_id': decision,
        'decision_after_goal_s': call['decision_stamp_ns'] / 1e9 - goal_time,
        'outcome_after_goal_s_clock_aligned': end_ns / 1e9 - goal_time,
        'input_goal_context_distance_m': context.get('goal_distance_m'),
        'input_supervisor_mode': context.get('supervisor_mode'),
        'input_loop_count': canonical['memory']['loop_warning'].get('repeated_branch_count'),
        'selected_candidate': selected['candidate_ref'],
        'candidate_goal_alignment': selected.get('goal_alignment'),
        'candidate_goal_bearing_error_deg': selected.get('goal_bearing_error_deg'),
        'candidate_distance_m': selected.get('distance_m'),
        'policy_actions_chronological': ' '.join(str(r['action']) for r in steps),
        'raw_argmax_matches_effective': all(max(range(6), key=r['raw_logits'].__getitem__)
                                          == r['action'] for r in steps),
        'outcome_translation_m': outcome['moved_distance_m'],
        'outcome_progress': outcome['progress'], 'outcome_no_progress': outcome['no_progress'],
        'outcome_supervisor_mode': outcome['supervisor_mode'],
        'terminal_reason': outcome['terminal_reason'],
        'terminal_goal_consistent': timing.get('goal_consistent'),
        'terminal_refresh_goal_distance_m': refresh.get('pointgoal_distance_m'),
    })
    if decision in critical:
        canonical_inputs.append({'decision_id': decision, 'input': canonical,
                                 'model_output': json.loads(call['content']),
                                 'apply_report': apply, 'outcome': outcome,
                                 'terminal_timing': timing, 'terminal_refresh': refresh})
# Preserve the final escape proposal even though it has no completed policy session.
for decision in critical - {r['decision_id'] for r in canonical_inputs}:
    call = nav_calls[decision]
    text = next(c['text'] for m in call['request']['messages'] if m['role'] == 'user'
                for c in m['content'] if c['type'] == 'text')
    canonical_inputs.append({'decision_id': decision,
                            'input': json.loads(text.split('VLM_INPUT_JSON:\n', 1)[1]),
                            'model_output': json.loads(call['content']),
                            'apply_report': applies[decision], 'outcome': None})

assert chain[6]['policy_actions_chronological'] == chain[7]['policy_actions_chronological'] == '0'
assert chain[7]['outcome_supervisor_mode'] == chain[8]['input_supervisor_mode'] == 'escape_deadlock'
assert chain[8]['candidate_goal_alignment'] == 'away_from_goal'
assert chain[8]['terminal_goal_consistent'] is False
assert chain[8]['outcome_progress'] is True
assert all(c['raw_argmax_matches_effective'] for c in chain)
result_events = [r for r in events if r['event'] == 'result']
results = [r['data'] for r in result_events]
assert len(results) == 60
assert sum(r['executed'] and r['status'] == 'EXECUTED' for r in results) == 59
failed_results = [r for r in result_events if not r['data']['executed']]
assert len(failed_results) == 1
assert failed_results[0]['data']['error_code'] == 'MOTION_INHIBITED'
assert failed_results[0]['elapsed_s'] >= stop_event['elapsed_s']
assert len(calls) == 121 and not any(r.get('error') for r in calls)
assert not any(float(r['distance_m']) <= 1 for r in distances)

direct = []
for repetition in range(2, 7):
    run = 'direct_goal-goal-4-%03d' % repetition
    p = js(night / 'episodes/recaptured5' / run / 'policy_outputs.json')
    last = p[-1]
    assert len({r['session_index'] for r in p}) == 1
    assert last['action'] == 0 and max(range(6), key=last['raw_logits'].__getitem__) == 0
    assert len(p) < 33
    direct.append({'run': run, 'policy_outputs': len(p), 'last_step_index_zero_based': last['step_index'],
                   'last_raw_argmax': 0, 'last_effective_action': last['action'],
                   'stop_logit_margin_to_next': last['raw_logits'][0] - max(last['raw_logits'][1:]),
                   'model_stop_before_history_limit': True,
                   'look_substitutions': sum(r['action'] in (4, 5) for r in p)})
assert sum(r['policy_outputs'] for r in direct) == 118
assert sum(r['look_substitutions'] for r in direct) == 31
plan = js(night / 'episodes/recaptured5/direct_goal-goal-4-002/evidence/goal-plan.json')
budgets = []
for goal in plan['goals']:
    xy = goal['point_goal_xy_m']
    distance = math.hypot(xy['x'], xy['y'])
    minimum = max(0, distance - 1.)
    budgets.append({'goal': goal['label'], 'euclidean_start_distance_m': distance,
                    'success_radius_m': 1., 'straight_distance_to_radius_m': minimum,
                    'nominal_translation_per_forward_m': .25, 'max_actions_early_stop_true': 32,
                    'nominal_all_forward_budget_m': 8.,
                    'min_nominal_forward_actions_to_radius': math.ceil(minimum / .25),
                    'straight_distance_exceeds_nominal_budget': minimum > 8.})
history = js(HERE / 'history_limit.json')
assert history['cases'][0]['first_stop_call_1based'] == 33
assert history['cases'][1]['error'].startswith('PixelNav history exhausted: 65')
assert history['cases'][2]['first_stop_call_1based'] == 33

save('critical_decision_evidence.json', canonical_inputs)
save('ours_goal2_chain.json', chain)
csvout('ours_goal2_chain.csv', chain)
csvout('direct_stops.csv', direct)
csvout('direct_nominal_horizon_budget.csv', budgets)
summary = {
    'scope': 'Historical evidence plus isolated history-wrapper test; no robot motion or runtime fix.',
    'ours_goal2': {
        'run': 'full-goal-2-006', 'goal_publish_utc': goal_event['utc'],
        'closest_goal_distance_m': float(closest['distance_m']),
        'closest_after_goal_s': float(closest['t_s']),
        'duration_to_stop_s': stop_event['elapsed_s'] - goal_event['elapsed_s'],
        'last_goal_distance_before_stop_m': float(distances[-1]['distance_m']),
        'completed_sessions': len(outcomes), 'action_results_captured': len(results),
        'executed_action_results': 59,
        'failed_result_after_direct_stop': failed_results[0],
        'vlm_requests': len(calls), 'vlm_request_errors': 0,
        'first_repeated_immediate_stop_sessions': [7, 8],
        'escape_session': chain[8],
        'escape_gates': [r for r in async_rows if r['event'] == 'pointnav_loop_escape_candidate_gate'],
        'policy_to_decision_join': '13 chronological session groups matched to 13 chronological completed decision outcomes; not an ID join.',
        'clock_alignment': 'Outcome monotonic delta added to same-process apply now_ns. CSV decision times use ROS wall stamps. Distances remain source/receipt measurements, not exact simultaneous ground truth.',
    },
    'direct': direct,
    'limits': ['No counterfactual physical trajectory or goal visibility ground truth.',
               'Candidate metadata angle is from the captured observation; it is not an exact executed body rotation.',
               '8m is nominal macro command budget, not a hard measured physical travel bound.',
               'History cap was not reached in the five historical Direct failures.',
               'A temporary increase in goal distance may be necessary for a valid detour.'],
}
save('summary.json', summary)

# Verify every consumed historical evidence file against its pre-existing seal.
for name, meta in provenance.items():
    bundle = name.split('/')[0]
    if bundle == 'pixnav_failure_followup':
        continue
    entries = {}
    for line in (ROOT / bundle / 'SHA256SUMS').read_text().splitlines():
        digest, rel = line.split(None, 1)
        rel = rel.lstrip('*')
        if rel.startswith('./'):
            rel = rel[2:]
        entries[rel] = digest
    relative = name.split('/', 1)[1]
    assert entries.get(relative) == meta['sha256'], ('sealed evidence mismatch', name)
save('input_provenance.json', provenance)
save('validation.json', {'passed': True, 'direct_model_stop_cases': 5,
                         'ours_policy_outputs_checked': len(policy), 'ours_outcomes_checked': len(outcomes),
                         'action_results_checked': len(results),
                         'executed_actions': 59, 'inhibited_after_timeout_stop': 1,
                         'consumed_evidence_files': len(provenance),
                         'historical_inputs_match_existing_seals': True,
                         'physical_commands_sent': 0, 'runtime_changes_applied': False})

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'pdf.fonttype': 42})
fig, ax = plt.subplots(figsize=(10, 4.6))
ax.plot([float(r['t_s']) for r in distances], [float(r['distance_m']) for r in distances],
        color='#2367a8', lw=1.5, label='기록된 최종 목표 거리')
ax.axhline(1, color='#287b45', ls='--', label='도착 반경 1m')
for index in (6, 7):
    ax.axvline(chain[index]['outcome_after_goal_s_clock_aligned'], color='#c98319', ls=':', lw=1,
               label='연속 즉시 STOP 종료' if index == 6 else None)
ax.axvspan(chain[8]['decision_after_goal_s'], chain[8]['outcome_after_goal_s_clock_aligned'],
           alpha=.16, color='#c8443d', label='탈출 후보 적용 → 정책 종료 (시계 정렬 근사)')
ax.annotate('최근접 1.253m', (float(closest['t_s']), float(closest['distance_m'])),
            xytext=(105, 2.45), arrowprops={'arrowstyle': '->'}, fontsize=10)
ax.annotate('이동량 2.46m → progress=true\n최종 목표 거리 약 3.71m',
            (chain[8]['outcome_after_goal_s_clock_aligned'], 3.707), xytext=(240, 4.6),
            arrowprops={'arrowstyle': '->'}, fontsize=10)
ax.set(xlabel='목표 발송 후 시간 (초)', ylabel='목표까지 오도메트리 거리 (m)',
       title='Ours 2번: 목표 근처 즉시 STOP → 탈출 후보 선택 → 거리 증가', xlim=(0, 362), ylim=(0, 6.2))
ax.grid(alpha=.2)
ax.legend(loc='upper right', fontsize=9)
fig.tight_layout()
fig.savefig(out / 'ours_goal2_failure_chain.png', dpi=160)
fig.savefig(out / 'ours_goal2_failure_chain.pdf')
plt.close(fig)
print(json.dumps({'output': str(out), 'passed': True, 'closest_m': float(closest['distance_m']),
                  'direct_stop_calls': [r['policy_outputs'] for r in direct]}, ensure_ascii=False))
