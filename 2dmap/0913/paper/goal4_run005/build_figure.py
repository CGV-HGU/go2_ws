#!/usr/bin/env python3
"""Render the requested recorded PixNav 005 example. No robot commands."""
import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.transforms import Bbox
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE.parent
MAP = OUT.parent
REPO = MAP.parents[1]
spec = importlib.util.spec_from_file_location('paper_style', MAP / 'goal4_comparison_paper/build_figure.py')
style = importlib.util.module_from_spec(spec)
spec.loader.exec_module(style)
evidence = style.evidence
STEM = 'fig6_goal4_ours_vs_pixnav_run005'
DEFAULT = 'fig6_goal4_ours_vs_pixnav_paper'


def check_scale_bounds(ax, artists, free, meta):
    """Check the complete text/stroke footprint, plus padding, against the map."""
    renderer = ax.figure.canvas.get_renderer()
    box = Bbox.union([artist.get_window_extent(renderer) for artist in artists])
    box = box.expanded(1.08, 1.15)
    corners = ax.transData.inverted().transform(box.get_points())
    x0, y0 = corners[0] - .05
    x1, y1 = corners[1] + .05
    # Include neighbouring cells to cover the interpolated vector contour edge.
    x = meta['min_x'] + np.arange(free.shape[1]) * meta['resolution']
    y = meta['max_y'] - np.arange(free.shape[0]) * meta['resolution']
    cells = free[np.ix_((y >= y0-.05) & (y <= y1+.05), (x >= x0-.05) & (x <= x1+.05))]
    assert cells.size > 0 and not cells.any(), 'Scale overlaps a free/white map region'
    assert ax.get_xlim()[0] < x0 < x1 < ax.get_xlim()[1]
    assert ax.get_ylim()[0] < y0 < y1 < ax.get_ylim()[1]
    origin = ax.transData.transform((0, 0))
    x_m = np.linalg.norm(ax.transData.transform((1, 0)) - origin)
    y_m = np.linalg.norm(ax.transData.transform((0, 1)) - origin)
    assert abs(x_m - y_m) < 1e-8
    return dict(complete_footprint_with_margin_xy_m=[x0, y0, x1, y1],
        inspected_map_cells=int(cells.size), overlapping_white_cells=int(cells.sum()),
        backing_rectangle=False, same_horizontal_vertical_scale=True,
        scale_length_m=1.0, scale_length_original_map_pixels=1/meta['resolution'])


def main():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
        'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False})
    meta = evidence.js(MAP / '2d_metadata.json')
    rgb = np.array(Image.open(io.BytesIO(evidence.read(MAP / '2d_clean_publication.png'))))
    free = np.all(rgb[:, :, :3] == 255, axis=2)
    ours = evidence.load_run('original5', 'full-goal-4-009')
    pix = evidence.load_run('recaptured5', 'direct_goal-goal-4-005')
    assert np.max(np.abs(ours['goal_xy']-pix['goal_xy'])) < 1e-10
    assert ours['binding']['map_sha256'] == pix['binding']['map_sha256']
    assert ours['result']['reason'] == 'GOAL_DISTANCE_REACHED'
    assert pix['result']['reason'] == 'PIXNAV_TERMINAL_BEFORE_GOAL'
    point_validation = {r['run']: style.verify_raw(r) for r in (ours, pix)}
    # Capture the actual goal-admission record, rather than inferring it from the plot.
    with gzip.open(io.BytesIO(evidence.read(pix['root'] / 'evidence/full5m/policy-trace/pixnav_apply_trace.jsonl.gz')), 'rt') as stream:
        apply = next(e for e in (json.loads(line) for line in stream) if e['event'] == 'apply_report')
    assert apply['apply_status'] == 'admit' and apply['reprojection_status'] == 'projectable'

    fig, ax = plt.subplots(figsize=(8.4, 6.55))
    fig.subplots_adjust(left=.025, right=.975, top=.985, bottom=.155)
    style.background(ax, free, meta)
    for run, color, name in [(ours, style.OURS_COLOR, 'ours'), (pix, style.PIX_COLOR, 'pixnav005')]:
        xy = run['xy']
        limits = style.LIMITS
        assert ((xy[:, 0] > limits[0]) & (xy[:, 0] < limits[1]) &
                (xy[:, 1] > limits[2]) & (xy[:, 1] < limits[3])).all()
        ax.plot(*xy.T, color=color, lw=2.6, zorder=4,
            solid_capstyle='round', solid_joinstyle='round', gid=name+'-recorded-trajectory')
        ax.plot(*xy[-1], marker='o', ms=8, mfc=style.FREE, mec=color,
            mew=1.8, ls='none', zorder=7, gid=name+'-final-position')
    ax.plot(0, 0, marker='o', ms=9, mfc='#18262D', mec='white', mew=1.1,
            ls='none', zorder=8, gid='start-position')
    goal = ours['goal_xy']
    ax.plot(*goal, marker='*', ms=16, mfc=style.GOAL_COLOR, mec='white',
            mew=1.2, ls='none', zorder=8, gid='goal4-marker')
    ax.text(goal[0]+.28, goal[1]-.32, 'Goal 4', color='#A52C26', fontsize=10.5,
            va='top', zorder=9, gid='goal4-label')

    # No white backing: bracket and text sit entirely on the existing gray area.
    sx, sy = 12.0, 3.20
    scale_line, = ax.plot([sx, sx, sx+1, sx+1], [sy+.12, sy, sy, sy+.12],
            color='#141414', lw=2.3, solid_joinstyle='miter', zorder=10, gid='scale-bar-1m')
    scale_text = ax.text(sx+.5, sy+.23, '1 m', ha='center', va='bottom',
            fontsize=14, zorder=10, gid='scale-label-1m')
    methods = [Line2D([], [], color=style.PIX_COLOR, lw=2.6, label='Direct-goal PixNav'),
               Line2D([], [], color=style.OURS_COLOR, lw=2.6, label='ESCAPE-Nav (Ours)')]
    markers = [Line2D([], [], ls='none', marker='o', ms=8, color='#18262D', label='Start position'),
               Line2D([], [], ls='none', marker='o', ms=8, mfc='white', mec='#222222', mew=1.6, label='Final position'),
               Line2D([], [], ls='none', marker='*', ms=14, color=style.GOAL_COLOR, label='Goal point')]
    fig.legend(handles=methods, loc='lower center', bbox_to_anchor=(.5, .064),
               ncol=2, frameon=False, fontsize=12, handlelength=3.1, columnspacing=2.1)
    fig.legend(handles=markers, loc='lower center', bbox_to_anchor=(.5, .004),
               ncol=3, frameon=False, fontsize=11, handlelength=1.4, columnspacing=1.6)
    fig.canvas.draw()
    scale_validation = check_scale_bounds(ax, [scale_line, scale_text], free, meta)
    for ext in ('png', 'pdf', 'svg'):
        path = OUT / (STEM+'.'+ext)
        fig.savefig(path, dpi=350, facecolor='white', bbox_inches='tight', pad_inches=.025)
        if ext == 'svg':
            # Cosmetic XML whitespace only; keep editable text and all vertices.
            path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines())+'\n')
        shutil.copyfile(path, OUT / (DEFAULT+'.'+ext))
    plt.close(fig)

    for run, name in [(ours, 'ours_trajectory.csv'), (pix, 'pixnav_trajectory.csv')]:
        (HERE/name).write_bytes(evidence.read(run['root'] / 'trajectory.csv'))
    selected = [dict(run=r['run'], campaign=r['campaign'],
        reason=r['result']['reason'], final_goal_distance_m=r['result']['goal_distance_m'],
        duration_s=r['metrics']['elapsed_to_stop_s'], path_length_10hz_m=r['metrics']['path_length']['10hz_m'],
        samples=len(r['xy']), field_report=r['field'], navigation_image=r['metrics']['images']['escape']) for r in (ours, pix)]
    metadata = dict(ours_run=ours['run'], pixnav_run=pix['run'], goal_label='4',
        goal_xy_m=goal.tolist(), selected_by='User requested candidate run 005, not Goal 5.',
        selected_metrics=selected, paper_style_reference_commit=style.PAPER_COMMIT,
        map_source='2dmap/0913/2d_clean_publication.png',
        map_overlay='Same contextual marked-start overlay as the original 0913 template; no new ICP registration.',
        map_processing='Existing free mask unchanged, including all holes; no new morphology or hand-drawn geometry.',
        trajectory_processing='All pre-stop exported odometry points in order, no reflection, smoothing, decimation or offset.',
        scale=scale_validation, default_paper_basename=DEFAULT, explicit_run_basename=STEM,
        goal_admission=dict(goal_published=pix['result']['goal_published'],
            apply_status=apply['apply_status'], reprojected_image_point=apply['reprojected_image_point'],
            navigation_target_xy=apply['navigation_target_xy'],
            meaning='Goal 4 coordinate and its pixel projection were admitted; this does not prove target visibility or camera calibration accuracy.'),
        independent_ground_truth=False, spl=None,
        limitations=['Selected qualitative example, not a cohort statistic.',
                    'Field intervention/contact/obstruction are unconfirmed for both selected runs.',
                    'Navigation image revisions differ; the overlay is start-aligned odometry, not independent ground truth.'])
    for file in [MAP/'goal4_comparison/build_figure.py', MAP/'goal4_comparison_paper/build_figure.py', Path(__file__).resolve()]:
        evidence.read(file)
    for name, data in [('metadata.json', metadata), ('validation.json', dict(points=point_validation, scale=scale_validation)),
                       ('input_provenance.json', evidence.INPUTS)]:
        (HERE/name).write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n')
    artifacts = list(HERE.iterdir()) + [OUT/(stem+'.'+ext) for stem in (STEM, DEFAULT) for ext in ('png', 'pdf', 'svg')]
    if (OUT/'README.md').exists(): artifacts.append(OUT/'README.md')
    (HERE/'SHA256SUMS').write_text(''.join(hashlib.sha256(file.read_bytes()).hexdigest()+'  '+str(file.relative_to(OUT))+'\n'
        for file in sorted(artifacts) if file.is_file() and file.name != 'SHA256SUMS'))
    print(json.dumps(dict(output=str(OUT/(STEM+'.png')), points=point_validation, scale=scale_validation), indent=2))


if __name__ == '__main__':
    main()
