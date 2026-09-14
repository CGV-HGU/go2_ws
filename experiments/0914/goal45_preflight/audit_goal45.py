#!/usr/bin/env python3
"""Read-only live-file audit. No launches, sensor subscriptions, or robot commands."""
import csv
import datetime as dt
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys
import types

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
W = HERE.parents[2]
R = Path('/home/unitree/s2e-vlm-async-framework-minimal')
C = R / '.local-data/recording-five-goals-recaptured5-20260914'
hashes = {}


def raw(p):
    p = Path(p)
    data = p.read_bytes()
    hashes[str(p)] = hashlib.sha256(data).hexdigest()
    return data


def js(p):
    return json.loads(raw(p))


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    raw(path)
    return value


sys.path.insert(0, str(R / 'scripts'))
fixed = module('fixed_goal45_audit', R / 'scripts/fixed_start_goals.py')
runner = module('runner_goal45_audit', R / 'scripts/run_saved_robot_goal.py')
c, plan = runner.load_campaign(C)
raw(C / 'campaign.json')
raw(C / 'goal-plan.json')
mapfile = Path(c['map_database'])
assert hashlib.sha256(raw(mapfile)).hexdigest() == c['map_sha256']
database = sqlite3.connect(mapfile.as_uri() + '?mode=ro')
database.execute('PRAGMA query_only=ON')
quick = database.execute('PRAGMA quick_check').fetchall()
assert quick == [('ok',)]
dbinfo = {'path': str(mapfile), 'sha256': c['map_sha256'], 'quick_check': 'ok',
          'nodes': database.execute('SELECT count(*) FROM Node').fetchone()[0],
          'link_rows': database.execute('SELECT count(*) FROM Link').fetchone()[0],
          'nonempty_ground_truth_blob_rows': database.execute(
              'SELECT count(*) FROM Node WHERE ground_truth_pose IS NOT NULL AND length(ground_truth_pose)>0').fetchone()[0]}
gt = [row[0] for row in database.execute('SELECT ground_truth_pose FROM Node')]
dbinfo['all_ground_truth_blobs_zero_filled'] = all(
    data is not None and len(data) == 48 and not np.any(np.frombuffer(data, dtype='<f4')) for data in gt)
dbinfo['external_ground_truth_available'] = False
database.close()

oldplan = js(HERE.parent / 'night_log_review/episodes/original5/full-goal-4-009/evidence/goal-plan.json')
retained = []
for label in ('1', '2', '3', '4'):
    old = fixed.validate_plan(oldplan, label)['point_goal_xy_m']
    new = fixed.validate_plan(plan, label)['point_goal_xy_m']
    error = math.hypot(new['x']-old['x'], new['y']-old['y'])
    assert error == 0
    retained.append({'label': label, 'difference_to_success_campaign_m': error})

source_capture = Path(c['goal_set_derivation']['new_goal_capture'])
cap = js(source_capture)
origin = js(source_capture.parent / 'origin.json')
snapshot = js(source_capture.parent / cap['source_snapshot_file'])
assert hashlib.sha256(raw(source_capture.parent / cap['source_snapshot_file'])).hexdigest() == cap['source_snapshot_sha256']
recomputed = fixed.capture_goal(origin, '5', snapshot, cap['captured_unix'], origin['host_boot_id'])
assert recomputed['point_goal_xy_m'] == cap['point_goal_xy_m'] == fixed.validate_plan(plan, '5')['point_goal_xy_m']
closure = js(C / 'provenance/return-closure.json')
registration = js(C / 'provenance/frame-registration.json')
overlay = js(W / '2dmap/0914/2d_goals_map_recaptured5.json')
assert overlay['goal_plan_sha256'] == c['goal_plan_sha256']
assert overlay['map_sha256'] == c['map_sha256']
review = js(R / '.local-data/recording-five-goals-20260913/map-review/review.json')
cells_file = Path(overlay['map_cells_file'])
cells = np.load(io.BytesIO(raw(cells_file)), allow_pickle=False)
assert hashlib.sha256(raw(cells_file)).hexdigest() == overlay['map_cells_sha256']
xmin, xmax, ymin, ymax = review['extent_xy']
resolution = (xmax-xmin)/cells.shape[1]
assert abs((ymax-ymin)/cells.shape[0]-resolution) < 1e-12
gy, gx = np.indices(cells.shape)
gridx, gridy = xmin+(gx+.5)*resolution, ymin+(gy+.5)*resolution
goals, transforms = [], []
for label in ('4', '5'):
    goal = fixed.validate_plan(plan, label)
    xy = goal['point_goal_xy_m']
    shown = next(g for g in overlay['goals'] if g['goal_label'] == label)
    assert xy == {'x': shown['fixed_start_x_m'], 'y': shown['fixed_start_y_m']}
    heading = overlay['map_overlay_heading_rad']
    mx = overlay['map_overlay_origin_xy'][0] + math.cos(heading)*xy['x'] - math.sin(heading)*xy['y']
    my = overlay['map_overlay_origin_xy'][1] + math.sin(heading)*xy['x'] + math.cos(heading)*xy['y']
    assert math.hypot(mx-shown['map_overlay_x_m'], my-shown['map_overlay_y_m']) < 1e-12
    ix, iy = math.floor((mx-xmin)/resolution), math.floor((my-ymin)/resolution)
    region = (gridx-mx)**2+(gridy-my)**2 <= 1.
    values, counts = np.unique(cells[region], return_counts=True)
    goals.append({'label': label, 'fixed_start_x_m': xy['x'], 'fixed_start_y_m': xy['y'],
                  'straight_distance_m': math.hypot(xy['x'], xy['y']),
                  'bearing_from_marked_forward_deg': math.degrees(math.atan2(xy['y'], xy['x'])),
                  'saved_yaw_rad_not_enforced': goal['yaw_rad'], 'map_overlay_xy_m': [mx, my],
                  'projected_cell_value': int(cells[iy, ix]),
                  'projected_cells_within_1m': {str(int(v)): int(n) for v, n in zip(values, counts)},
                  'physical_goal_accuracy_verified': False})
    for start in [(0., 0., 0.), (3., -2., math.pi/2), (-5., 2., -math.pi/2),
                  (3., 4., math.pi), (1.4, -3.7, .606), (100., -100., -2.9)]:
        dispatched = fixed.goal_in_odometry(plan, label, start)
        dx, dy = dispatched[0]-start[0], dispatched[1]-start[1]
        recovered = [math.cos(start[2])*dx+math.sin(start[2])*dy,
                     -math.sin(start[2])*dx+math.cos(start[2])*dy]
        error = math.hypot(recovered[0]-xy['x'], recovered[1]-xy['y'])
        assert error < 1e-12
        transforms.append({'label': label, 'episode_start': start, 'dispatch_xy': dispatched,
                           'roundtrip_error_m': error})

# Existing validator renders configurations and checks image/config hashes but
# normally rewrites manifest flags. Suppress that single write in an in-memory
# copy so the prepared, immutable episode files remain untouched.
validator_path = R / 'scripts/validate_robot_pointgoal_trial.py'
text = raw(validator_path).decode()
mutation = "    (root/'manifest.json').write_text(json.dumps(m,indent=2)+'\\n')"
assert text.count(mutation) == 1
v = types.ModuleType('goal45_readonly_validator')
v.__file__ = str(validator_path)
exec(compile(text.replace(mutation, '    pass  # Audit: manifest write suppressed'), str(validator_path), 'exec'), v.__dict__)
prepared = []
for name in ('full-goal-4-004', 'full-goal-5-002', 'direct_goal-goal-4-007', 'direct_goal-goal-5-002'):
    path = C / 'episodes' / name
    before = raw(path / 'manifest.json')
    result = v.validate(path, c['validation'])
    assert before == (path / 'manifest.json').read_bytes()
    mf = json.loads(before)
    assert raw(path/'goal-plan.json') == raw(C/'goal-plan.json')
    prepared.append({'run': name, 'validation': result, 'artifact_check_only': True,
                     'conditions': mf['runtime_conditions'], 'images': mf['images'],
                     'manifest_unchanged': True})

free = shutil.disk_usage(C).free
duration = c['trial_settings']['max_duration_s']
required = runner.required_recording_space_bytes(duration)
rows = list(csv.DictReader(io.StringIO(raw(HERE.parent/'night_log_review/episodes_summary.csv').decode())))
history = [r for r in rows if r['navigation'] == 'full' and r['goal'] == '4']
result = {'checked_at_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
          'map': dbinfo, 'goals': goals, 'retained_goals': retained,
          'goal5_capture_recomputed_exactly': True, 'transforms': transforms,
          'return_closure': closure, 'frame_registration': registration,
          'prepared_trials': prepared, 'ours_goal4_history': history,
          'recording_space': {'free_bytes': free, 'episode_max_duration_s': duration,
                              'runner_required_bytes': required,
                              'one_episode_preflight_space_passes': free >= required,
                              'two_worst_case_episodes_bytes_including_one_reserve': 10*1024**3+2*duration*30*1024**2,
                              'two_worst_case_episodes_fit': free >= 10*1024**3+2*duration*30*1024**2},
          'remap_for_current_fixed_start_navigation_required': False,
          'remap_reason': 'Frozen DB is unchanged and structurally valid; navigation uses fixed-start odometry. This does not verify global localization, physical goal accuracy, current obstacles, or SPL reference quality.',
          'physical_commands_sent': 0, 'map_or_goal_files_changed': False,
          'runtime_source_changed': False,
          'checks_passed': True, 'real_drive_ready': False,
          'input_sha256': hashes}
(HERE/'goal45_audit.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
print(json.dumps({'checks_passed': True, 'goals': goals, 'map': dbinfo,
                  'prepared_count': len(prepared), 'recording_space': result['recording_space']},ensure_ascii=False))
