#!/usr/bin/env python3
"""Offline episode evidence/export. No ROS, network, or command interfaces.

Raw files remain at the immutable per-trial directory. The optional tar bundle
copies those exact bytes. Odom length and map-based SPL are estimates, not GT.
"""
import argparse
import bisect
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import tarfile


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def read_rows(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def pose_row(event):
    msg = event['data']
    p = msg['pose']['pose']
    q = p['orientation']
    st = msg['header']['stamp']
    values = (event['elapsed_s'], p['position']['x'], p['position']['y'],
              math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                         1-2*(q['y']**2+q['z']**2)))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite trajectory')
    return dict(elapsed_s=values[0], x_m=values[1], y_m=values[2], yaw_rad=values[3],
                stamp_ns=st['sec']*10**9+st['nanosec'], frame=msg['header']['frame_id'])


def path_lengths(rows, start, end):
    """Interpolate at fixed receipt-time intervals; keep raw length for sensitivity.

    Never bridge an odometry reset or long recording gap. Do not join map pose
    corrections into physical travel. End is the first direct StopMove record.
    Post-stop coasting is outside this estimate and must not be called measured
    settled-endpoint path length.
    """
    if end <= start or len(rows) < 2:
        raise ValueError('Missing trajectory interval')
    if not all(math.isfinite(v) for v in (start, end)) or not all(
            math.isfinite(r[k]) for r in rows for k in ('elapsed_s', 'x_m', 'y_m')):
        raise ValueError('Non-finite trajectory')
    t = [r['elapsed_s'] for r in rows]
    if any(b <= a for a, b in zip(t, t[1:])):
        raise ValueError('Non-monotonic trajectory receipt times')
    first = max(0, bisect.bisect_right(t, start)-1)
    last = min(len(t)-1, bisect.bisect_left(t, end))
    rows = rows[first:last+1]
    t = [r['elapsed_s'] for r in rows]
    if t[0] > start or end-t[-1] > .15:
        raise ValueError('Trajectory does not cover episode endpoints')
    actual_end = min(end, t[-1])
    if len({r['frame'] for r in rows}) != 1 or rows[0]['frame'] != 'odom':
        raise ValueError('Mixed/incorrect odometry frame')
    for a, b in zip(rows, rows[1:]):
        dt = b['elapsed_s']-a['elapsed_s']
        if b['stamp_ns'] <= a['stamp_ns']:
            raise ValueError('Odometry timestamp reset/duplicate')
        if dt > .5:
            raise ValueError('Odometry recording gap exceeds 0.5s')
        if math.hypot(b['x_m']-a['x_m'], b['y_m']-a['y_m']) > .1+1.5*dt:
            raise ValueError('Odometry discontinuity')

    def point(at):
        j = min(len(t)-2, max(0, bisect.bisect_right(t, at)-1))
        f = (at-t[j])/(t[j+1]-t[j])
        return tuple(rows[j][k]+f*(rows[j+1][k]-rows[j][k]) for k in ('x_m', 'y_m'))

    def length(times):
        p = [point(v) for v in times]
        return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(p, p[1:]))

    result = {}
    for hz in (5, 10, 20):
        times = [start+i/hz for i in range(int((actual_end-start)*hz)+1)]
        if times[-1] < actual_end:
            times.append(actual_end)
        result[str(hz)+'hz_m'] = length(times)
    result['raw_samples_m'] = length([start]+[v for v in t if start < v < actual_end]+[actual_end])
    result['last_sample_before_stop_s'] = end-actual_end
    result['max_receipt_gap_s'] = max(b-a for a, b in zip(t, t[1:]))
    result['source'] = 'continuous odom XY, fixed receipt-time resampling, no denoising'
    result['boundary'] = 'goal publication to direct StopMove request; post-stop settling excluded'
    return result


def success_label(result, field):
    if not result or not result.get('goal_published'):
        return None
    if result.get('reason') != 'GOAL_DISTANCE_REACHED':
        return 0
    if result.get('shutdown_service_confirmed') is not True:
        return 0
    d, radius = result.get('goal_distance_m'), result.get('success_distance_m')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (d, radius)):
        return None
    if not 0 <= d <= radius or radius <= 0:
        return 0
    if not field or field.get('direct_intervention') is None:
        return None  # Never silently turn an unreviewed finish into a success.
    if result.get('localization_method') == 'fixed_start_odometry':
        if field['direct_intervention'] is not False:
            return 0
        # The fixed-start odom threshold alone cannot establish arrival at a
        # physical saved goal. Keep borderline/unmeasured field reports pending.
        within = field.get('physical_within_goal_radius_verified')
        return int(within) if isinstance(within, bool) else None
    return int(field['direct_intervention'] is False)


def spl(success, shortest, travelled):
    if success not in (0, 1) or not all(math.isfinite(v) for v in (shortest, travelled)):
        raise ValueError('Invalid SPL inputs')
    if shortest <= 0 or travelled < 0:
        raise ValueError('Degenerate shortest path or invalid travel length')
    return success*shortest/max(shortest, travelled)


def reference_length(reference, goal, manifest):
    """Require an explicit, inspected reference bound to this map and endpoints."""
    if not reference or reference.get('reviewed') is not True:
        raise ValueError('Reviewed shortest-path reference not supplied')
    if goal['frame'] != 'map' or reference.get('frame') != 'map':
        raise ValueError('Reference requires a map-frame trial')
    expected = manifest.get('evaluation_map_sha256')
    if not expected or reference.get('map_sha256') != expected:
        raise ValueError('Reference map binding missing/mismatched')
    for label, actual in [('start_xy', [goal['origin_x'], goal['origin_y']]),
                          ('goal_xy', [goal['goal_x'], goal['goal_y']])]:
        target = reference.get(label, [])
        if len(target) != 2 or math.hypot(target[0]-actual[0], target[1]-actual[1]) > .01:
            raise ValueError('Reference endpoints differ from actual episode')
    if reference.get('method') not in ('reviewed_static_map_shortest_path', 'surveyed_shortest_path'):
        raise ValueError('Mapping walk/Euclidean distance is not a shortest-path reference')
    value = reference.get('distance_m')
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('Invalid reference distance')
    if value+.01 < math.hypot(goal['goal_x']-goal['origin_x'], goal['goal_y']-goal['origin_y']):
        raise ValueError('Reference shorter than Euclidean lower bound')
    return value


def percentile(values, q):
    v = sorted(values)
    return v[min(len(v)-1, int(q*(len(v)-1)))] if v else None


def analyze(run, reference=None):
    run = Path(run)
    rt = run/'full5m/runtime'
    events = read_rows(rt/'events.jsonl')
    result = read_json(rt/'result.json')
    goal = read_json(rt/'goal.json')
    manifest = read_json(run/'manifest.json', {})
    field = read_json(run/'field-report.json')
    starts = [e for e in events if e['event'] == 'goal_published']
    if len(starts) > 1:
        raise ValueError('Multiple goals in one episode directory')
    episode_ids = {e['data']['episode_id'] for e in events if e['event'] == 'task'
                   and (not starts or e['elapsed_s'] >= starts[0]['elapsed_s'])}
    if len(episode_ids) > 1:
        raise ValueError('Multiple task episodes in one directory')
    row = dict(run=str(run.resolve()), episode_id=next(iter(episode_ids), None),
               started=bool(starts), result_reason=(result or {}).get('reason', 'INCOMPLETE'),
               success_radius_m=(result or {}).get('success_distance_m'),
               success=success_label(result, field), field_report=field,
               path_length=None, spl_estimate=None, metric_warnings=[],
               ground_truth_available=False, arrival_mode='automatic_distance_stop',
               images=manifest.get('images'), runtime_conditions=manifest.get('runtime_conditions'))
    if starts and not result:
        row['metric_warnings'].append('Started episode lacks final result; outcome pending, never omitted')
    trajectories = {kind: [pose_row(e) for e in events if e['event'] == kind]
                    for kind in ('odom', 'localization')}
    if starts:
        start = starts[0]['elapsed_s']
        stops = [e['elapsed_s'] for e in events if e['event'] == 'direct_stop' and e['elapsed_s'] >= start]
        end = stops[0] if stops else None
        row['elapsed_to_stop_s'] = end-start if end is not None else None
        if end is not None:
            try:
                row['path_length'] = path_lengths(trajectories['odom'], start, end)
            except ValueError as exc:
                row['metric_warnings'].append(str(exc))
        else:
            row['metric_warnings'].append('No recorded StopMove endpoint')
        for values in trajectories.values():
            for value in values:
                value['episode_time_s'] = value['elapsed_s']-start
        pg = [e for e in events if e['event'] == 'pointgoal' and e['elapsed_s'] >= start
              and e['data'].get('episode_id') in episode_ids]
        row['minimum_recorded_goal_distance_m'] = min((e['data']['distance_m'] for e in pg), default=None)
        first = next((e for e in pg if e['data']['distance_m'] <= e['data']['success_distance_m']), None)
        row['first_radius_entry_s'] = first['elapsed_s']-start if first else None
    if reference and goal:
        try:
            row['reference_distance_m'] = reference_length(reference, goal, manifest)
            if row['success'] is not None and row['path_length']:
                row['spl_estimate'] = spl(row['success'], row['reference_distance_m'], row['path_length']['10hz_m'])
        except ValueError as exc:
            row['metric_warnings'].append(str(exc))
    else:
        row['metric_warnings'].append('Shortest path unmeasured; SPL left unset')
    calls = read_rows(run/'full5m/policy-trace/vlm_calls.jsonl')
    row['vlm_http'] = {}
    for kind in sorted({x.get('call_type', 'unknown') for x in calls}):
        selected = [x for x in calls if x.get('call_type', 'unknown') == kind]
        v = [x['latency_s'] for x in selected if isinstance(x.get('latency_s'), (int, float))]
        row['vlm_http'][kind] = dict(count=len(selected), errors=sum(bool(x.get('error')) for x in selected),
                                   p50_s=percentile(v, .5), p95_s=percentile(v, .95), max_s=max(v) if v else None)
    row['vlm_timing_note'] = 'HTTP latency per request, not complete decision or exclusive waiting time'
    row['action_results'] = dict(collections.Counter(e['data']['action']+':'+e['data']['status']
                                                   for e in events if e['event'] == 'result'))
    return row, trajectories


def archive(run, output, reference=None, bundle=False):
    run, output = Path(run).resolve(), Path(output).resolve()
    if run == output or run in output.parents:
        raise ValueError('Output must be outside the raw episode directory')
    if output.exists():
        raise ValueError('Use a new archive output directory')
    files = []
    for p in sorted(run.rglob('*')):
        if p.is_symlink():
            raise ValueError('Symlink in raw episode; archive explicit files only')
        if not p.is_file():
            continue
        before = p.stat()
        h = digest(p)
        after = p.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError('Raw episode is still changing')
        files.append(dict(path=str(p.relative_to(run)), bytes=after.st_size,
                          mtime_ns=after.st_mtime_ns, sha256=h))
    row, trajectories = analyze(run, reference)
    for entry in files:
        st = (run/entry['path']).stat()
        if (st.st_size, st.st_mtime_ns) != (entry['bytes'], entry['mtime_ns']):
            raise ValueError('Raw episode changed during analysis')
    output.mkdir(parents=True)
    for kind, values in trajectories.items():
        if values:
            with (output/(kind+'-trajectory.csv')).open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(values[0])); w.writeheader(); w.writerows(values)
    index = dict(schema='escape-robot-episode-archive-v1', raw_root=str(run), files=files,
                 total_bytes=sum(x['bytes'] for x in files), raw_files_rewritten=False,
                 coordinate_sources=['continuous odom', 'accepted map localization when available'])
    if reference:
        (output/'reference-path.json').write_text(json.dumps(reference, indent=2)+'\n')
    (output/'episode.json').write_text(json.dumps(row, indent=2, ensure_ascii=False)+'\n')
    (output/'raw-manifest.json').write_text(json.dumps(index, indent=2)+'\n')
    if bundle:
        with tarfile.open(str(output/'raw-episode.tar.gz'), 'w:gz') as t:
            for entry in files:
                t.add(str(run/entry['path']), arcname=entry['path'], recursive=False)
        # Confirm that the copied bytes are the ones indexed above.
        with tarfile.open(str(output/'raw-episode.tar.gz'), 'r:gz') as t:
            for entry in files:
                h = hashlib.sha256()
                with t.extractfile(entry['path']) as f:
                    for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
                if h.hexdigest() != entry['sha256']:
                    raise ValueError('Bundle/source mismatch')
    return row


def aggregate(rows):
    attempts = [r for r in rows if r['started']]
    ids = [r['episode_id'] for r in attempts]
    if None in ids or len(set(ids)) != len(ids):
        raise ValueError('Missing/duplicate episode ID; keep attempts individually identifiable')
    conditions = set()
    for row in attempts:
        runtime = {k: v for k, v in (row.get('runtime_conditions') or {}).items()
                   if k not in ('goal_label', 'goal_distance_m')}
        conditions.add(json.dumps([row.get(k) for k in ('success_radius_m', 'arrival_mode', 'images')]
                                  +[runtime], sort_keys=True))
    if len(conditions) > 1:
        raise ValueError('Mixed configurations/radii; aggregate separate cohorts')
    n = len(attempts)
    successes = sum(r['success'] == 1 for r in attempts)
    pending = sum(r['success'] is None for r in attempts)
    missing_spl = sum(r['spl_estimate'] is None for r in attempts)
    return dict(episodes=n, prestart_records=len(rows)-n, successes=successes, pending_outcomes=pending,
                sr=successes/n if n and not pending else None,
                sr_lower=successes/n if n else None, sr_upper=(successes+pending)/n if n else None,
                spl_estimate=sum(r['spl_estimate'] for r in attempts)/n if n and not missing_spl else None,
                missing_spl_episodes=missing_spl)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir', type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--reference', type=Path)
    p.add_argument('--bundle', action='store_true')
    p.add_argument('--aggregate', nargs='+', type=Path, metavar='EPISODE_JSON')
    args = p.parse_args()
    if args.aggregate:
        if args.run_dir or args.reference or args.bundle: p.error('Aggregation cannot archive a run')
        result = aggregate([read_json(path) for path in args.aggregate])
        with args.output.open('x') as f: json.dump(result, f, indent=2)
    else:
        if not args.run_dir: p.error('--run-dir is required')
        result = archive(args.run_dir, args.output, read_json(args.reference) if args.reference else None, args.bundle)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
