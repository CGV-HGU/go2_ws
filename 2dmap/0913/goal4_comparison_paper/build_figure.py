#!/usr/bin/env python3
"""Offline, evidence-backed Goal 4 figure in the manuscript's ObjectNav style."""
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
from matplotlib.patches import FancyBboxPatch
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
MAP = HERE.parent
REPO = HERE.parents[2]
spec = importlib.util.spec_from_file_location('evidence', MAP / 'goal4_comparison/build_figure.py')
evidence = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evidence)

STEM = 'fig6_goal4_ours_vs_pixnav_paper'
PAPER_COMMIT = '2ccd9e7761bb3e29384a8c4f3d02056e855cddd5'
OURS_COLOR, PIX_COLOR = '#009E73', '#AF49A3'
BACKGROUND, FREE, EDGE, GOAL_COLOR = '#D8DFE3', '#FCFDFD', '#BFC8CE', '#D83B32'
LIMITS = (-1.8, 13.5, -6.3, 4.1)


def verify_raw(run):
    """Independently recover every plotted point from the immutable event stream."""
    compressed = evidence.read(run['root'] / 'evidence/full5m/runtime/events.jsonl.gz')
    points, times, start = [], [], None
    goal = run['goal']
    c, s = math.cos(goal['heading_rad']), math.sin(goal['heading_rad'])
    with gzip.open(io.BytesIO(compressed), 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            if event['event'] == 'goal_published' and start is None:
                start = event['elapsed_s']
            elif start is not None and event['event'] == 'direct_stop':
                break
            elif start is not None and event['event'] == 'odom':
                p = event['data']['pose']['pose']['position']
                dx, dy = p['x'] - goal['origin_x'], p['y'] - goal['origin_y']
                points.append([c * dx + s * dy, -s * dx + c * dy])
                times.append(event['elapsed_s'] - start)
    points, times = np.array(points), np.array(times)
    assert points.shape == run['xy'].shape
    xy_error = float(np.max(np.abs(points - run['xy'])))
    time_error = float(np.max(np.abs(times - run['t'])))
    assert xy_error < 1e-12 and time_error < 1e-12
    return dict(samples=len(points), max_xy_difference_m=xy_error,
                max_time_difference_s=time_error, all_original_samples_preserved=True)


def background(ax, free, meta, limits=LIMITS):
    # Same pixel-centre registration as the original 0913 figure. Rendering only:
    # retain the existing publication map's entire free mask, including its holes.
    h, w = free.shape
    x = meta['min_x'] + np.arange(w) * meta['resolution']
    y = meta['max_y'] - np.arange(h) * meta['resolution']
    ax.set_facecolor(BACKGROUND)
    ax.contourf(x, y, free.astype(float), levels=[.5, 1.5], colors=[FREE],
                antialiased=True, zorder=0)
    ax.contour(x, y, free.astype(float), levels=[.5], colors=[EDGE],
               linewidths=.55, antialiased=True, zorder=1)
    ax.set_xlim(*limits[:2]); ax.set_ylim(*limits[2:])
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color('#111111'); spine.set_linewidth(1.2)


def trajectories(ax, ours, pix, width=2.6, marker_size=8):
    for run, color in [(ours, OURS_COLOR), (pix, PIX_COLOR)]:
        xy = run['xy']
        # No smoothing, decimation, fitted bends, or position offsets.
        ax.plot(xy[:, 0], xy[:, 1], color=color, lw=width, zorder=4,
                solid_capstyle='round', solid_joinstyle='round')
        ax.plot(*xy[-1], marker='o', ms=marker_size, mfc=FREE, mec=color,
                mew=1.8, ls='none', zorder=7)
    ax.plot(0, 0, marker='o', ms=marker_size + 1, mfc='#18262D', mec='white',
            mew=1.1, ls='none', zorder=8)
    ax.plot(*ours['goal_xy'], marker='*', ms=marker_size + 8, mfc=GOAL_COLOR,
            mec='white', mew=1.2, ls='none', zorder=8)


def render(ours, pix, free, meta, stem):
    fig, ax = plt.subplots(figsize=(8.4, 6.55))
    fig.subplots_adjust(left=.025, right=.975, top=.985, bottom=.155)
    background(ax, free, meta)
    trajectories(ax, ours, pix)
    goal = ours['goal_xy']
    ax.text(goal[0] + .28, goal[1] - .32, 'Goal 4', color='#A52C26',
            fontsize=10.5, va='top', zorder=9)
    # Exactly one metre in metric data coordinates, with the reference's bracket.
    sx, sy = 11.65, 3.25
    ax.add_patch(FancyBboxPatch((sx - .25, sy - .22), 1.5, 1.0,
        boxstyle='round,pad=0.05,rounding_size=0.10', fc=FREE, ec='none', zorder=9))
    ax.plot([sx, sx, sx + 1, sx + 1], [sy + .12, sy, sy, sy + .12],
            color='#141414', lw=2.3, solid_joinstyle='miter', zorder=10)
    ax.text(sx + .5, sy + .23, '1 m', ha='center', va='bottom', fontsize=14, zorder=10)
    methods = [Line2D([], [], color=PIX_COLOR, lw=2.6, label='Direct-goal PixNav'),
               Line2D([], [], color=OURS_COLOR, lw=2.6, label='ESCAPE-Nav (Ours)')]
    markers = [Line2D([], [], ls='none', marker='o', ms=8, color='#18262D', label='Start position'),
               Line2D([], [], ls='none', marker='o', ms=8, mfc='white', mec='#222222', mew=1.6, label='Final position'),
               Line2D([], [], ls='none', marker='*', ms=14, color=GOAL_COLOR, label='Goal point')]
    fig.legend(handles=methods, loc='lower center', bbox_to_anchor=(.5, .064),
               ncol=2, frameon=False, fontsize=12, handlelength=3.1, columnspacing=2.1)
    fig.legend(handles=markers, loc='lower center', bbox_to_anchor=(.5, .004),
               ncol=3, frameon=False, fontsize=11, handlelength=1.4, columnspacing=1.6)
    fig.canvas.draw()
    origin = ax.transData.transform((0, 0))
    dx = np.linalg.norm(ax.transData.transform((1, 0)) - origin)
    dy = np.linalg.norm(ax.transData.transform((0, 1)) - origin)
    assert abs(dx - dy) < 1e-8 and 1 / meta['resolution'] == 20
    for run in (ours, pix):
        xy = run['xy']
        assert ((xy[:, 0] > LIMITS[0]) & (xy[:, 0] < LIMITS[1]) &
                (xy[:, 1] > LIMITS[2]) & (xy[:, 1] < LIMITS[3])).all()
    for ext in ('png', 'pdf', 'svg'):
        fig.savefig(MAP / (stem + '.' + ext), dpi=350, facecolor='white',
                    bbox_inches='tight', pad_inches=.025)
    plt.close(fig)


def main():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none', 'path.simplify': False})
    meta = evidence.js(MAP / '2d_metadata.json')
    rgb = np.array(Image.open(io.BytesIO(evidence.read(MAP / '2d_clean_publication.png'))))
    free = np.all(rgb[:, :, :3] == 255, axis=2)
    ours = evidence.load_run('original5', 'full-goal-4-009')
    candidates = [evidence.load_run('recaptured5', 'direct_goal-goal-4-%03d' % i)
                  for i in range(2, 7)]
    # Prefer the strongest recorded progress, rather than a more dramatic failure.
    pix = min(candidates, key=lambda r: r['result']['goal_distance_m'])
    assert pix['run'] == 'direct_goal-goal-4-003'
    for run in candidates:
        assert np.max(np.abs(run['goal_xy'] - ours['goal_xy'])) < 1e-10
        assert run['binding']['map_sha256'] == ours['binding']['map_sha256']
        assert run['result']['reason'] == 'PIXNAV_TERMINAL_BEFORE_GOAL'
        assert run['result']['success_distance_m'] == 1.0
    assert ours['result']['reason'] == 'GOAL_DISTANCE_REACHED'
    assert ours['result']['goal_distance_m'] <= 1.0
    validation = {r['run']: verify_raw(r) for r in (ours, pix)}
    render(ours, pix, free, meta, STEM)
    # Retain the earlier alternative in the same style for transparent selection.
    alternate = next(r for r in candidates if r['run'].endswith('-004'))
    validation[alternate['run']] = verify_raw(alternate)
    render(ours, alternate, free, meta, STEM + '_alternate004')
    for r, name in [(ours, 'ours_trajectory.csv'), (pix, 'pixnav_trajectory.csv')]:
        (HERE / name).write_bytes(evidence.read(r['root'] / 'trajectory.csv'))
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5))
    for ax, run in zip(axes.flat, candidates):
        background(ax, free, meta)
        trajectories(ax, ours, run, width=1.3, marker_size=4)
        ax.set_title(run['run'].rsplit('-', 1)[-1] + ('  [selected]' if run is pix else ''), fontsize=11)
        ax.text(.03, .03, 'Final distance: %.2f m' % run['result']['goal_distance_m'],
                transform=ax.transAxes, fontsize=8, bbox=dict(fc=FREE, ec='none', alpha=.95))
    axes.flat[-1].axis('off')
    axes.flat[-1].text(.04, .8,
        '003: closest to Goal 4 among five runs.\nForward travel followed by a right turn.\n\nPurple: Direct-goal PixNav\nGreen: ESCAPE-Nav (same run in all panels)',
        transform=axes.flat[-1].transAxes, fontsize=10, va='top', linespacing=1.7)
    fig.tight_layout(); fig.savefig(HERE / 'pixnav_candidates.png', dpi=180); plt.close(fig)
    metadata = dict(ours_run=ours['run'], pixnav_run=pix['run'], goal_labels_displayed=['4'],
        goal_xy_m=ours['goal_xy'].tolist(), goal_radius_m=1.0, goal_radius_drawn=False,
        scale_bar_m=1.0, scale_bar_map_pixels=20, equal_axes=True, limits_m=LIMITS,
        paper_reference_commit=PAPER_COMMIT,
        paper_reference='paper/figures/svg/fig6_scalebars.pdf',
        paper_caption='Matched ObjectNav example: ESCAPE-Nav reaches the goal; PixNav and VOCA remain locally confined.',
        map_source='2dmap/0913/2d_clean_publication.png',
        displayed_free_mask_cells=int(free.sum()),
        map_rendering='Unmodified white/free mask from the existing publication map, including all holes; gray non-free area and 0.5-level vector outlines. No new morphological filtering or hand-drawn walls.',
        map_overlay='Same identity/pixel-centre registration as the requested 0913 template; contextual overlay, not new ICP registration.',
        trajectory_rendering='Every exported pre-stop odometry XY point in order. No smoothing, decimation, offset, or deformation.',
        selection='003 has the smallest final goal distance and greatest 10 Hz path length of the five recorded alternatives; it includes four executed right turns.',
        candidates=[dict(run=r['run'], final_distance_m=r['result']['goal_distance_m'],
            path_length_10hz_m=r['metrics']['path_length']['10hz_m'],
            duration_s=r['metrics']['elapsed_to_stop_s'],
            action_results=r['metrics']['action_results']) for r in candidates],
        selected=[dict(run=r['run'], result=r['result']['reason'],
            final_goal_distance_m=r['result']['goal_distance_m'],
            path_length_10hz_m=r['metrics']['path_length']['10hz_m'],
            duration_s=r['metrics']['elapsed_to_stop_s'], samples=len(r['xy']),
            field_report=r['field'], navigation_image=r['metrics']['images']['escape']) for r in (ours, pix)],
        independent_ground_truth=False, spl=None)
    for file in sorted((HERE / 'reference').iterdir()):
        if file.is_file(): evidence.read(file)
    evidence.read(MAP / 'goal4_comparison/build_figure.py')
    for name, value in [('figure_metadata.json', metadata), ('validation.json', validation),
                         ('input_provenance.json', evidence.INPUTS)]:
        (HERE / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    artifacts = list(HERE.rglob('*')) + list(MAP.glob(STEM + '.*')) + list(MAP.glob(STEM + '_alternate004.*'))
    lines = []
    for file in sorted(artifacts):
        if not file.is_file() or file.name == 'SHA256SUMS' or '__pycache__' in file.parts:
            continue
        lines.append(hashlib.sha256(file.read_bytes()).hexdigest() + '  ' + str(file.relative_to(MAP)))
    (HERE / 'SHA256SUMS').write_text('\n'.join(lines) + '\n')
    print(json.dumps(dict(main=str(MAP / (STEM + '.png')), selected=pix['run'],
        verified_raw_points=validation,
        previous_manifest_files_verified=sum(v['matches_prior_evidence_manifest'] for v in evidence.INPUTS.values())), indent=2))


if __name__ == '__main__':
    main()
