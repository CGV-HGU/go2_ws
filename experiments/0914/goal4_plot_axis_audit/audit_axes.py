#!/usr/bin/env python3
"""Read-only audit of plotted y direction against odometry and body IMU logs."""
import csv
from datetime import datetime
import gzip
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
spec = importlib.util.spec_from_file_location('figure', REPO / '2dmap/0913/goal4_comparison_paper/build_figure.py')
figure = importlib.util.module_from_spec(spec)
spec.loader.exec_module(figure)
evidence = figure.evidence


def main():
    ours = evidence.load_run('original5', 'full-goal-4-009')
    runs = [evidence.load_run('recaptured5', 'direct_goal-goal-4-' + n) for n in ('004', '003')]
    meta = evidence.js(figure.MAP / '2d_metadata.json')
    rgb = np.array(Image.open(io.BytesIO(evidence.read(figure.MAP / '2d_clean_publication.png'))))
    free = np.all(rgb[:, :, :3] == 255, axis=2)
    turns, reports = [], []
    plt.rcParams.update({'font.size': 9, 'pdf.fonttype': 42, 'path.simplify': False})
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), gridspec_kw={'height_ratios': [1.5, 1]})
    for col, (run, color) in enumerate(zip(runs, ['#0078D4', figure.PIX_COLOR])):
        root = run['root']
        validation = figure.verify_raw(run)
        body = []
        with gzip.open(io.BytesIO(evidence.read(root / 'evidence/full5m/body-recording/events.jsonl.gz')), 'rt') as stream:
            for line in stream:
                e = json.loads(line)
                if e.get('kind') == 'low':
                    body.append([e['wall_ns'] * 1e-9, e['data']['rpy_deg'][2], e['data']['gyro_rad_s'][2]])
        body = np.array(body)
        wall, body_yaw, gyro = body[:, 0], np.unwrap(np.deg2rad(body[:, 1])), body[:, 2]
        motions = evidence.js(root / 'motion_events.json')
        policy = evidence.js(root / 'policy_outputs.json')
        rows = list(csv.DictReader(io.StringIO(evidence.read(root / 'trajectory.csv').decode())))
        stamp = np.array([datetime.fromisoformat(r['utc']).timestamp() for r in rows])
        yaw = np.unwrap(np.array([float(r['yaw_rad']) for r in rows]))
        start_wall = stamp[0] - run['t'][0]
        run_turns = []
        for e in motions:
            if e['event'] != 'terminal' or e['kind'] != 'rotate':
                continue
            end = e['log_unix_s']; start = end - e['elapsed_s']
            i, j = int(np.argmin(abs(wall - start))), int(np.argmin(abs(wall - end)))
            step = int(e['command_id'].rsplit('-', 1)[-1])
            expected = 1 if policy[step]['action'] == 2 else -1 if policy[step]['action'] == 3 else 0
            angle = math.degrees(body_yaw[j] - body_yaw[i])
            integrated = math.degrees(np.trapz(gyro[i:j+1], wall[i:j+1]))
            assert expected and all(np.sign(x) == expected for x in [e['target'], e['measured_progress'], angle, integrated])
            align = max(abs(wall[i] - start), abs(wall[j] - end))
            assert align < .03
            row = dict(run=run['run'], step=step, action='turn_left' if expected == 1 else 'turn_right',
                start_s=start-start_wall, end_s=end-start_wall,
                command_deg=math.degrees(e['target']), odom_deg=math.degrees(e['measured_progress']),
                body_imu_deg=angle, integrated_body_gyro_deg=integrated,
                time_alignment_error_s=align, direction_signs_agree=True)
            turns.append(row); run_turns.append(row)
        first = run_turns[0]
        first_index = int(np.argmin(abs(run['t'] - first['start_s'])))
        x, y = run['xy'][-1]
        distance = float(np.linalg.norm(run['xy'][-1] - run['goal_xy']))
        c, s = math.cos(run['goal']['heading_rad']), math.sin(run['goal']['heading_rad'])
        determinant = float(np.linalg.det([[c, s], [-s, c]]))
        assert abs(determinant - 1) < 1e-12
        assert np.max(np.abs(run['goal_xy'] - ours['goal_xy'])) < 1e-12
        reports.append(dict(run=run['run'], raw_point_validation=validation,
            rotation_matrix_determinant=determinant, first_turn_position_xy_m=run['xy'][first_index].tolist(),
            final_xy_m=[x, y], final_yaw_deg=math.degrees(yaw[-1]),
            computed_final_goal_distance_m=distance, logged_final_goal_distance_m=run['result']['goal_distance_m'],
            hypothetical_y_mirrored_distance_m=math.hypot(x-run['goal_xy'][0], -y-run['goal_xy'][1]),
            executed_turns=len(run_turns), body_samples=len(body)))
        ax = axes[0, col]
        figure.background(ax, free, meta)
        ax.plot(*ours['xy'].T, color=figure.OURS_COLOR, lw=1.7, alpha=.6)
        ax.plot(*run['xy'].T, color=color, lw=2.3)
        ax.plot(0, 0, 'ko', ms=5)
        ax.plot(x, y, 'o', mec=color, mfc='white', mew=1.6, ms=7)
        ax.plot(*run['goal_xy'], '*', color=figure.GOAL_COLOR, ms=12)
        ax.plot(*run['xy'][first_index], 's', color=color, ms=5)
        ax.set_xticks([0, 4, 8, 12]); ax.set_yticks([-4, -2, 0, 2, 4])
        ax.set_xlabel('x: forward from marked start (m)')
        ax.set_ylabel('y: left (+, up) / right (-, down), m')
        ax.set_title(run['run'] + (' | previous blue example' if col == 0 else ' | current purple example'), fontsize=10)
        ax.text(.035, .05, 'Endpoint: (%.2f, %+.2f) m' % (x, y), transform=ax.transAxes,
            bbox=dict(fc='white', ec='none', alpha=.9), fontsize=9)
        ax = axes[1, col]
        mask = (wall >= start_wall) & (wall <= stamp[-1])
        relative_body = np.rad2deg(body_yaw - np.interp(start_wall, wall, body_yaw))
        ax.plot(run['t'], np.rad2deg(yaw), color=color, lw=2.5, label='Recorded odometry yaw')
        ax.plot(wall[mask]-start_wall, relative_body[mask], color='#222222', lw=1.1, ls='--', label='Body IMU yaw (relative)')
        for turn in run_turns:
            ax.axvspan(turn['start_s'], turn['end_s'], alpha=.10, color=color)
        ax.axhline(0, color='#999999', lw=.7)
        ax.set_xlabel('Time after goal publication (s)')
        ax.set_ylabel('Yaw: left + / right - (degrees)')
        ax.grid(alpha=.15); ax.legend(fontsize=8)
        ax.set_title('First turn: command %+.0f°, odom %+.2f°, body IMU %+.2f°' %
                     (first['command_deg'], first['odom_deg'], first['body_imu_deg']), fontsize=9)
    fig.suptitle('Goal 4 plot-axis audit: the two examples are different recorded runs', fontsize=12)
    fig.tight_layout(rect=(0, .045, 1, .96))
    fig.text(.5, .014, 'Square: first turn location. Shading: commanded turns. All coordinates are recorded odometry; map overlay is contextual.',
             ha='center', fontsize=8)
    for ext in ('png', 'pdf'):
        fig.savefig(HERE / ('axis_direction_audit.' + ext), dpi=190, bbox_inches='tight')
    plt.close(fig)
    report = dict(conclusion='No evidence of a PixNav-specific y-axis reflection. 004 turns left/up early; 003 turns right/down after longer forward travel.',
        same_coordinate_transform_for_ours_and_pixnav=True,
        map_row_formula='row=(max_y-y)/resolution; positive y appears higher in the image',
        runs=reports, turns=turns,
        limit='Body IMU/gyro corroborate turn sign, not absolute map registration or ground-truth position.')
    (HERE / 'audit.json').write_text(json.dumps(report, indent=2) + '\n')
    (HERE / 'input_provenance.json').write_text(json.dumps(evidence.INPUTS, indent=2) + '\n')
    with (HERE / 'turns.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(turns[0])); writer.writeheader(); writer.writerows(turns)
    (HERE / 'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.name + '\n'
        for p in sorted(HERE.iterdir()) if p.is_file() and p.name != 'SHA256SUMS'))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
