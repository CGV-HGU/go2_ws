#!/usr/bin/env python3
"""Recompute a descriptive Direct-goal PixelNav cohort from immutable exports.

No ROS, robot APIs, sockets, policy inference, or experiment mutations.
Requires the sibling night_log_review and pixnav_direction_review bundles.
Use --output for a separate reproduction directory; source evidence is read-only.
"""
import argparse
import collections
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
REVIEW = BASE / 'night_log_review'
DIRECTION = BASE / 'pixnav_direction_review'
ACT = ['stop', 'forward', 'left', 'right', 'look_up', 'look_down']
INPUTS = {}


def read_bytes(path):
    data = path.read_bytes()
    name = str(path.relative_to(BASE))
    INPUTS[name] = {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
    return data


def read_json(path):
    return json.loads(read_bytes(path))


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def write_csv(path, rows):
    assert rows
    with path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def assert_close(a, b, tolerance=1e-6):
    assert math.isfinite(a) and math.isfinite(b) and abs(a-b) <= tolerance, (a, b)


def independent_path(events, start, stop):
    """Independent numpy interpolation of the original receipt-time odom series."""
    rows = []
    for e in events:
        if e['event'] != 'odom':
            continue
        msg = e['data']; p = msg['pose']['pose']['position']; stamp = msg['header']['stamp']
        rows.append((e['elapsed_s'], p['x'], p['y'], stamp['sec']*10**9+stamp['nanosec'], msg['header']['frame_id']))
    t = np.array([r[0] for r in rows]); xy = np.array([[r[1], r[2]] for r in rows])
    assert len(t) > 1 and np.isfinite(xy).all() and np.isfinite(t).all()
    assert (np.diff(t) > 0).all()
    a = max(0, np.searchsorted(t, start, side='right') - 1)
    b = min(len(t)-1, np.searchsorted(t, stop, side='left'))
    sel = rows[a:b+1]; t = t[a:b+1]; xy = xy[a:b+1]
    assert t[0] <= start and stop-t[-1] <= .15
    assert {r[4] for r in sel} == {'odom'}
    assert all(b[3] > a[3] for a, b in zip(sel, sel[1:]))
    gaps = np.diff(t)
    assert (gaps <= .5).all()
    assert (np.linalg.norm(np.diff(xy, axis=0), axis=1) <= .1+1.5*gaps).all()
    end = min(stop, t[-1]); lengths = {}
    for hz in (5, 10, 20):
        grid = start + np.arange(int((end-start)*hz)+1)/hz
        if grid[-1] < end:
            grid = np.append(grid, end)
        points = np.column_stack([np.interp(grid, t, xy[:, j]) for j in (0, 1)])
        lengths[str(hz)+'hz_m'] = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    lengths['max_receipt_gap_s'] = float(gaps.max())
    lengths['stop_endpoint_coverage_gap_s'] = max(0., stop-float(t[-1]))
    return lengths


def stats(rows, key):
    x = np.array([r[key] for r in rows], dtype=float)
    return {'mean': float(x.mean()), 'median': float(np.median(x)),
            'sample_sd': float(x.std(ddof=1)) if len(x) > 1 else None,
            'min': float(x.min()), 'max': float(x.max())}


def save(fig, output, name):
    fig.savefig(output / (name+'.png'), dpi=180, bbox_inches='tight')
    fig.savefig(output / (name+'.pdf'), bbox_inches='tight')
    plt.close(fig)


def main(output):
    output.mkdir(parents=True, exist_ok=True)
    summaries = read_json(REVIEW / 'episodes_summary.json')
    clarifications = read_json(REVIEW / 'operator_clarifications.json')
    facts = {(x['campaign'], x['run']): x for x in clarifications['confirmations']}
    direction = read_json(DIRECTION / 'direction_results.json')
    replay = read_json(DIRECTION / 'replay_results.json')
    assert replay['all_actions_match'] is True
    # Capture source attestations without treating them as new hardware tests.
    source_validation = read_json(REVIEW / 'validation.json')
    assert source_validation['passed'] is True
    direction_rows = {x['run']: x for x in direction['runs']}
    direct_all = [r for r in summaries if r['navigation'] == 'direct_goal']
    started = sorted((r for r in direct_all if r['started']), key=lambda r: r['start_kst'])
    excluded = [r for r in direct_all if not r['started']]
    assert len(started) == 5 and len(excluded) == 1
    assert {r['goal'] for r in started} == {'4'}
    assert {r['campaign'] for r in direct_all} == {'recaptured5'}
    inventory = []; rows = []; paths = {}; policy_counts = collections.Counter()
    checked_actions = 0
    for index, meta in enumerate(started, 1):
        run = meta['run']; root = REVIEW / 'episodes' / meta['campaign'] / run
        evdata = read_bytes(root / 'evidence/full5m/runtime/events.jsonl.gz')
        events = [json.loads(s) for s in gzip.decompress(evdata).splitlines() if s.strip()]
        result = read_json(root / 'evidence/full5m/runtime/result.json')
        goal = read_json(root / 'evidence/full5m/runtime/goal.json')
        manifest = read_json(root / 'evidence/manifest.json')
        recomputed = read_json(root / 'episode.recomputed.json')
        original_field = read_json(root / 'evidence/field-report.json')
        policy = read_json(root / 'policy_outputs.json')
        motion = read_json(root / 'motion_events.json')
        traj = list(csv.DictReader(read_bytes(root / 'trajectory.csv').decode().splitlines()))
        control = list(csv.DictReader(read_bytes(root / 'control_pose.csv').decode().splitlines()))
        start_events = [e for e in events if e['event'] == 'goal_published']
        assert len(start_events) == 1
        start = start_events[0]['elapsed_s']
        stop = next(e['elapsed_s'] for e in events if e['event'] == 'direct_stop' and e['elapsed_s'] >= start)
        duration = stop-start
        episode_ids = {e['data']['episode_id'] for e in events if e['event'] == 'task' and e['elapsed_s'] >= start}
        assert len(episode_ids) == 1
        pg = [e for e in events if e['event'] == 'pointgoal' and start <= e['elapsed_s']
              and e['data'].get('episode_id') in episode_ids]
        assert pg
        distances = [e['data']['distance_m'] for e in pg]
        radius = result['success_distance_m']
        assert result['goal_published'] is True and result['shutdown_service_confirmed'] is True
        assert result['reason'] == 'PIXNAV_TERMINAL_BEFORE_GOAL'
        assert radius == 1 and result['max_duration_s'] == 360
        assert min(distances) > radius
        assert_close(result['goal_distance_m'], meta['final_goal_distance_m'])
        assert_close(result['goal_distance_m'], distances[-1])
        # The final result snapshot can include a callback received during stop
        # shutdown. Preserve this measured offset instead of forcing identical endpoints.
        final_distance_offset = pg[-1]['elapsed_s']-stop
        assert abs(final_distance_offset) < .1
        pg_before_stop = [e for e in pg if e['elapsed_s'] <= stop]
        assert_close(min(e['data']['distance_m'] for e in pg_before_stop), min(distances))
        assert_close(min(distances), meta['minimum_goal_distance_m'])
        assert_close(duration, meta['duration_s'])
        path = independent_path(events, start, stop)
        for hz in (5, 10, 20):
            assert_close(path[str(hz)+'hz_m'], meta['odom_path_'+str(hz)+'hz_m'])
            assert_close(path[str(hz)+'hz_m'], recomputed['path_length'][str(hz)+'hz_m'])
        assert [p['step_index'] for p in policy] == list(range(len(policy)))
        assert {p['session_index'] for p in policy} == {1}
        assert policy[-1]['action'] == 0 and policy[-1]['stop_source'] == 'model'
        assert all(p['action'] != 0 for p in policy[:-1])
        assert all(p['raw_logits'] == p['effective_logits'] for p in policy)
        assert all(int(np.argmax(p['raw_logits'])) == p['action'] for p in policy)
        terminal = [e for e in motion if e['event'] == 'terminal']
        assert len(terminal) == len(policy)-1
        for e in terminal:
            step = int(e['command_id'].rsplit('-', 1)[-1]); action = policy[step]['action']
            expected = math.pi/6 if action == 2 else -math.pi/6 if action == 3 else .2 if action in (4, 5) else .25
            assert e['status'] == 'OK'
            assert e['kind'] == ('rotate' if action in (2, 3) else 'translate')
            assert_close(e['target'], expected)
            assert e['target']*e['measured_progress'] > 0
        checked_actions += len(terminal)
        counts = collections.Counter(ACT[p['action']] for p in policy)
        policy_counts.update(counts)
        c = manifest['runtime_conditions']
        assert c['navigation'] == 'direct_goal' and c['look_execution'] == 'forward_0p2'
        assert c['forward_speed_mps'] == .5 and c['turn_speed_radps'] == .8
        assert c['max_duration_s'] == 360 and c['direct_goal_max_policy_steps'] == 500
        assert c['full_observation_sweeps_enabled'] is False and meta['vlm_calls'] == 0
        assert c['icp_corrections_used'] is False and manifest['final_yaw_requested'] is False
        assert meta['recording_integrity_passed'] is True
        xy = goal['fixed_start_goal_xy']; d0 = math.hypot(xy['x'], xy['y'])
        assert_close(d0, goal['distance_m'])
        assert np.allclose([xy['x'], xy['y']], direction_rows[run]['goal_xy_m'], atol=1e-8, rtol=0)
        fact = facts.get((meta['campaign'], run), {})
        confirmed = all(fact.get(k) is False for k in ('direct_intervention', 'contact', 'obstruction'))
        field_status = 'confirmed_no_intervention_contact_obstruction' if confirmed else 'unconfirmed'
        final = result['goal_distance_m']
        row = dict(run=run, goal_label=4, repetition=index, start_kst=meta['start_kst'],
                   start_to_goal_euclidean_m=d0, duration_to_stop_s=duration,
                   final_goal_distance_odom_m=final, min_goal_distance_odom_m=min(distances),
                   final_distance_receipt_relative_to_stop_s=final_distance_offset,
                   goal_distance_reduction_m=d0-final, goal_distance_reduction_fraction=(d0-final)/d0,
                   odom_path_5hz_m=path['5hz_m'], odom_path_10hz_m=path['10hz_m'], odom_path_20hz_m=path['20hz_m'],
                   path_over_whole_duration_mps=path['10hz_m']/duration,
                   policy_outputs=len(policy), executed_actions=len(terminal),
                   forward=counts['forward'], left=counts['left'], right=counts['right'],
                   look_up=counts['look_up'], look_down=counts['look_down'], stop=counts['stop'],
                   recorded_success=0, visited_radius_1m=0, result_reason=result['reason'],
                   recording_integrity_passed=True, field_context=field_status,
                   direct_intervention=fact.get('direct_intervention'), contact=fact.get('contact'),
                   obstruction=fact.get('obstruction'), physical_final_distance_m=None,
                   reviewed_geodesic_shortest_path_m=None, paper_spl=None,
                   failure_spl_contribution_if_admitted=0,
                   max_odom_receipt_gap_s=path['max_receipt_gap_s'],
                   endpoint_coverage_gap_s=path['stop_endpoint_coverage_gap_s'],
                   max_duration_s=360, forward_limit_mps=.5, turn_limit_radps=.8, look_translation_m=.2,
                   navigation_image=manifest['images']['escape'], checkpoint_sha256=manifest['checkpoint']['sha256'])
        rows.append(row)
        inventory.append(dict(campaign=meta['campaign'], run=run, goal_label=4, goal_published=True,
                              driving_denominator=True, field_context=field_status, reason=result['reason']))
        # Preserve receipt times and pose source stamps. The standard CSV is local-start odom,
        # not external GT or loop-closure-corrected map coordinates.
        td = output / 'trajectories'; td.mkdir(exist_ok=True)
        write_csv(td / (run+'.csv'), traj)
        pd = output / 'goal_distances'; pd.mkdir(exist_ok=True)
        pg_rows = [dict(t_s=e['elapsed_s']-start, utc=e['utc'], distance_m=e['data']['distance_m'],
                        bearing_rad=e['data']['bearing_rad']) for e in pg]
        write_csv(pd / (run+'.csv'), pg_rows)
        paths[run] = dict(trajectory=traj, pointgoal=pg_rows, goal=[xy['x'], xy['y']])
    assert checked_actions == 113
    assert sum(policy_counts.values()) == 118
    assert sum(r['field_context'].startswith('confirmed') for r in rows) == 1
    for r in excluded:
        prestart = read_json(REVIEW/'episodes'/r['campaign']/r['run']/'evidence/full5m/runtime/result.json')
        assert prestart['goal_published'] is False and prestart['reason'] == r['result_reason']
        inventory.append(dict(campaign=r['campaign'], run=r['run'], goal_label=int(r['goal']),
                              goal_published=False, driving_denominator=False,
                              field_context='not_applicable_prestart', reason=r['result_reason']))
    aggregate = dict(
        title='Direct-goal PixelNav: five repeated goal-4 episodes, descriptive real-robot results',
        schema_version=1, actual_attempts=6, started_episodes=5, prestart_failures=1,
        distinct_goals=1, goal_labels=[4], recorded_successes=0, recorded_sr=0.,
        visited_radius_successes=0, confirmed_no_intervention_contact_obstruction=1,
        field_context_unconfirmed=4, confirmed_subset_recorded_sr=0.,
        formal_paper_cohort_finalized=False, formal_paper_spl=None,
        failure_spl_contribution_if_all_five_admitted=0.,
        spl_note='Every recorded S is zero. Their SPL terms are algebraically zero if admitted under this declared protocol. No geodesic reference was measured; do not label this a complete independently validated SR/SPL benchmark.',
        recorded_protocol='Fixed-start odometry Euclidean radius 1 m with automatic arrival termination; final yaw unscored; 360 s historical cap.',
        statistics={k: stats(rows, k) for k in ['duration_to_stop_s', 'final_goal_distance_odom_m',
                    'min_goal_distance_odom_m', 'odom_path_10hz_m', 'goal_distance_reduction_fraction', 'path_over_whole_duration_mps']},
        policy_action_counts=dict(policy_counts), executed_actions_verified=checked_actions,
        model_stop_count=5, controller_action_failures=0, vlm_calls=0,
        episode_300s_or_360s_timeouts=0, raw_effective_action_mismatches=0,
        interpretation_limits=['One repeated target, not five independent goals.',
            'Four field reports are unknown; keep them in the descriptive cohort with their uncertainty.',
            'Distances and path lengths are odometry estimates, not ground truth.',
            'Occluded long-range final goal is projected into one initial image; goal pixel does not encode hidden distance.',
            'The same Go2 adapter changes look to 20 cm translation; this is not unmodified native PixelNav deployment.',
            'No performance evidence for Direct goal 1, 2, 3 or 5 in the reviewed campaigns.'])
    write_csv(output / 'episodes.csv', rows)
    write_csv(output / 'attempt_inventory.csv', inventory)
    write_json(output / 'episodes.json', rows)
    write_json(output / 'summary.json', aggregate)
    # Retain all outcomes across revisions for goals 1..3, rather than presenting only successes.
    goal123 = [{k: r.get(k) for k in ['campaign', 'run', 'goal', 'navigation', 'started', 'start_kst',
               'result_reason', 'duration_s', 'final_goal_distance_m', 'minimum_goal_distance_m',
               'odom_path_10hz_m', 'turn_speed_radps', 'look_execution', 'navigation_image',
               'recording_integrity_passed', 'recording_issues', 'field_report_status']}
               for r in summaries if r['goal'] in ('1', '2', '3')]
    write_csv(output / 'ours_goal123_history.csv', goal123)
    # The recent same-image triplet is a descriptive reference, never a paired SR superiority test.
    latest_ids = {'full-goal-1-007', 'full-goal-2-006', 'full-goal-3-008'}
    latest = [r for r in goal123 if r['run'] in latest_ids]
    assert len(latest) == 3 and len({r['navigation_image'] for r in latest}) == 1
    write_json(output / 'ours_goal123_same_revision_reference.json', latest)

    plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 10,
                         'pdf.fonttype': 42})
    colors = plt.cm.tab10(np.arange(5))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), gridspec_kw={'width_ratios': [1, 1.35]})
    for row, color in zip(rows, colors):
        d = paths[row['run']]; tr = d['trajectory']; pg = d['pointgoal']; label = row['run'][-3:]
        axes[0].plot([float(p['x_m']) for p in tr], [float(p['y_m']) for p in tr], c=color, label=label, lw=1.8)
        axes[0].plot(float(tr[-1]['x_m']), float(tr[-1]['y_m']), 'x', c=color, ms=7)
        axes[1].plot([p['t_s'] for p in pg], [p['distance_m'] for p in pg], c=color, label=label)
        axes[1].plot(pg[-1]['t_s'], pg[-1]['distance_m'], 'x', c=color, ms=7)
    goal = paths[rows[0]['run']]['goal']
    axes[0].plot(*goal, '*', c='black', ms=13, label='목표 4')
    axes[0].add_patch(plt.Circle(goal, 1., fill=False, color='black', ls='--'))
    axes[0].plot(0, 0, 'ko', label='표시 시작점')
    axes[0].set(xlabel='시작 방향 전방 x (m)', ylabel='시작 방향 좌측 y (m)', title='회차별 오도메트리 궤적 · 동일 축척')
    axes[0].set_aspect('equal', adjustable='box')
    axes[1].axhline(1, color='black', ls='--', label='도착 반경 1 m')
    axes[1].set(xlabel='목표 발송 후 시간 (초)', ylabel='추정 목표 거리 (m)', title='모든 회차가 모델 STOP으로 종료', ylim=(0, 12))
    for ax in axes:
        ax.grid(alpha=.2); ax.legend(fontsize=8, ncol=2)
    fig.suptitle('Direct-goal PixelNav · 4번 반복 5회 · 기록상 도착 0/5\n좌표·거리는 외부 실측이 아님 / 현장 무개입·무접촉 확인: 002만', fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .88)); save(fig, output, 'goal4_quantitative')

    # Human-readable Markdown table generated from the full-precision machine-readable rows.
    table = ['| 회차 | 소요 시간(s) | 이동 경로(m) | 최종 거리(m) | 최근접 거리(m) | 실행 동작 / 회전 | 현장 조건 |',
             '|---|---:|---:|---:|---:|---:|---|']
    for r in rows:
        table.append('| {} | {:.2f} | {:.2f} | {:.2f} | {:.2f} | {} / {} | {} |'.format(
            r['run'][-3:], r['duration_to_stop_s'], r['odom_path_10hz_m'], r['final_goal_distance_odom_m'],
            r['min_goal_distance_odom_m'], r['executed_actions'], r['left']+r['right'],
            '개입·접촉·방해 없음 확인' if r['field_context'].startswith('confirmed') else '미확인'))
    (output / 'episode_table.md').write_text('\n'.join(table)+'\n')
    # Every consumed prior artifact must match its sealed bundle, not just a freshly taken hash.
    expected = {}
    for bundle in (REVIEW, DIRECTION):
        for line in (bundle/'SHA256SUMS').read_text().splitlines():
            digest, name = line.split(maxsplit=1)
            expected[str((bundle/name.lstrip('*')).relative_to(BASE))] = digest
    missing = [p for p in INPUTS if p not in expected]
    mismatch = [p for p, m in INPUTS.items() if p in expected and expected[p] != m['sha256']]
    assert not missing and not mismatch, (missing, mismatch)
    write_json(output / 'input_provenance.json', INPUTS)
    write_json(output / 'validation.json', dict(
        checked_at_utc=datetime.now(timezone.utc).isoformat(), passed=True,
        source_hashes_verified_against_previous_sealed_bundles=len(INPUTS),
        source_hash_mismatches=[], independent_event_metric_recomputations=5,
        path_sampling_rates_hz=[5, 10, 20], prior_metric_tolerance=1e-6,
        policy_outputs_checked=118, executed_action_mappings_checked=113,
        prestart_failure_excluded_only_from_navigation_denominator=True,
        unconfirmed_field_reports_retained=4, real_robot_commands_sent=0,
        runtime_configuration_changed=False,
        scope='Recomputed logged outcomes, lengths and timings; no new physical test or counterfactual policy replay.'))
    print(json.dumps({'episodes': len(rows), 'sr': aggregate['recorded_sr'],
          'mean_final_distance_m': aggregate['statistics']['final_goal_distance_odom_m']['mean'],
          'mean_duration_s': aggregate['statistics']['duration_to_stop_s']['mean'],
          'source_files_verified': len(INPUTS)}, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=HERE)
    main(parser.parse_args().output.resolve())
