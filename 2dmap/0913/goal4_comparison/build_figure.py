#!/usr/bin/env python3
"""Render recorded Goal 4 trajectories. Offline; no robot or policy commands."""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.lines import Line2D
from matplotlib.patches import Circle
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
MAP = REPO / '2dmap/0913'
REVIEW = REPO / 'experiments/0914/night_log_review'
QUANT = REPO / 'experiments/0914/pixnav_quantitative_evaluation'
OURS = 'full-goal-4-009'
PIX = 'direct_goal-goal-4-003'
STEM = 'fig6_goal4_ours_vs_pixnav'
ORANGE, BLUE, GREEN = '#E67E22', '#0078D4', '#107C41'
INPUTS = {}
CHECKSUMS = {}
for bundle in (REVIEW, QUANT):
    for line in (bundle / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ', 1)
        CHECKSUMS[(bundle / name).resolve()] = digest


def read(path):
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if path.resolve() in CHECKSUMS:
        assert digest == CHECKSUMS[path.resolve()], ('Source hash mismatch', path)
    INPUTS[str(path.relative_to(REPO))] = dict(sha256=digest, bytes=len(data),
        matches_prior_evidence_manifest=(path.resolve() in CHECKSUMS))
    return data


def js(path):
    return json.loads(read(path))


def load_run(campaign, run):
    root = REVIEW / 'episodes' / campaign / run
    rows = list(csv.DictReader(io.StringIO(read(root / 'trajectory.csv').decode())))
    xy = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
    t = np.array([float(r['t_s']) for r in rows])
    assert len(rows)>1 and np.isfinite(xy).all() and np.isfinite(t).all()
    assert (np.diff(t)>0).all()
    assert {r['frame'] for r in rows} == {'episode_start'}
    assert {r['source'] for r in rows} == {'odom'}
    goal = js(root / 'evidence/full5m/runtime/goal.json')
    result = js(root / 'evidence/full5m/runtime/result.json')
    metrics = js(root / 'episode.recomputed.json')
    binding = js(root / 'evidence/campaign-binding.json')
    field = js(root / 'evidence/field-report.json')
    assert goal['goal_label'] == '4' and result['goal_label'] == '4'
    assert result['success_distance_m'] == 1.0
    assert t[0]>=0 and t[-1] <= metrics['elapsed_to_stop_s']
    goal_xy = np.array([goal['fixed_start_goal_xy'][k] for k in ('x','y')])
    assert np.linalg.norm(xy[0])<0.01
    return dict(run=run, campaign=campaign, root=root, xy=xy, t=t, goal_xy=goal_xy,
                goal=goal, result=result, metrics=metrics, binding=binding, field=field)


def add_arrows(ax, xy, color, fractions, length=0.25):
    # Arrows follow the observed polyline and do not add trajectory segments.
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    unique = np.r_[True, np.diff(arc)>1e-9]
    arc, points = arc[unique], xy[unique]
    for f in fractions:
        end = f*arc[-1]
        start = max(0, end-length)
        a = [np.interp(start, arc, points[:,j]) for j in (0,1)]
        b = [np.interp(end, arc, points[:,j]) for j in (0,1)]
        if np.linalg.norm(np.subtract(b,a))<0.10:
            continue
        ax.annotate('', xy=b, xytext=a, zorder=8,
            arrowprops=dict(arrowstyle='-|>', lw=1.8, color=color,
                           mutation_scale=15, shrinkA=0, shrinkB=0))


def background(ax, image, meta, limits):
    # Exact pixel registration used by the supplied 0913 template:
    # column=(x-min_x)/resolution; row=(max_y-y)/resolution.
    res = meta['resolution']; h,w = image.shape[:2]
    extent = [meta['min_x']-res/2, meta['min_x']+(w-.5)*res,
              meta['max_y']-(h-.5)*res, meta['max_y']+res/2]
    ax.imshow(image, origin='upper', extent=extent, interpolation='nearest', zorder=0)
    ax.set_xlim(*limits[:2]); ax.set_ylim(*limits[2:])
    ax.set_aspect('equal', adjustable='box'); ax.axis('off')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, default=MAP)
    args=parser.parse_args(); out=args.output_dir.resolve(); out.mkdir(parents=True,exist_ok=True)
    details=out/'goal4_comparison'; details.mkdir(exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'pdf.fonttype':42,
        'svg.fonttype':'none','path.simplify':False,'axes.titleweight':'bold'})
    meta=js(MAP/'2d_metadata.json')
    image=np.array(Image.open(io.BytesIO(read(MAP/'2d_wall_only.png'))))
    read(MAP/'fig6_wall_only_trajectories.png')
    read(REPO/'scratch/generate_0913_trajectories_on_map.py')
    ours=load_run('original5',OURS)
    candidates=[load_run('recaptured5',f'direct_goal-goal-4-{i:03d}') for i in range(2,7)]
    pix=next(r for r in candidates if r['run']==PIX)
    for r in candidates:
        assert np.max(np.abs(r['goal_xy']-ours['goal_xy']))<1e-10
        assert r['binding']['map_sha256']==ours['binding']['map_sha256']
        assert r['result']['reason']=='PIXNAV_TERMINAL_BEFORE_GOAL'
    assert ours['result']['reason']=='GOAL_DISTANCE_REACHED'
    assert ours['result']['goal_distance_m']<=1
    goal=ours['goal_xy']; radius=1.0
    # Context crop includes each selected path, both ends, and the entire goal disk.
    limits=(-2.0,14.1,-7.2,4.5)
    for r in (ours,pix):
        assert ((r['xy'][:,0]>limits[0]) & (r['xy'][:,0]<limits[1]) &
                (r['xy'][:,1]>limits[2]) & (r['xy'][:,1]<limits[3])).all()
    assert limits[0]<goal[0]-radius and goal[0]+radius<limits[1]
    assert limits[2]<goal[1]-radius and goal[1]+radius<limits[3]
    fig,ax=plt.subplots(figsize=(10.8,8.35))
    background(ax,image,meta,limits)
    for r,color,arrows in [(ours,ORANGE,[.30,.68,.91]),(pix,BLUE,[.18,.70])]:
        xy=r['xy']
        ax.plot(xy[:,0],xy[:,1],color=color,lw=3.0,zorder=6,
            solid_capstyle='round',solid_joinstyle='round',
            path_effects=[pe.Stroke(linewidth=4.5,foreground='white'),pe.Normal()])
        add_arrows(ax,xy,color,arrows)
    ax.add_patch(Circle(goal,radius,fill=False,ec=ORANGE,lw=1.5,ls=(0,(5,3)),zorder=4))
    ax.scatter(*goal,s=290,marker='*',color=ORANGE,edgecolor='#222222',lw=1.15,zorder=9)
    ax.text(goal[0]+.34,goal[1]-.28,'Goal 4',ha='left',va='top',fontsize=12.5,
        color='#B95300',fontweight='bold',zorder=10,
        bbox=dict(boxstyle='round,pad=.24',fc='white',ec='none',alpha=.95))
    for r,color,marker in [(ours,ORANGE,'P'),(pix,BLUE,'X')]:
        ax.scatter(*r['xy'][-1],s=150,marker=marker,color=color,edgecolor='white',lw=1.6,zorder=10)
    ax.scatter(0,0,s=170,color=GREEN,edgecolor='white',lw=1.7,zorder=11)
    ax.text(-.18,-.47,'Start',ha='left',va='top',fontsize=12,fontweight='bold',
        color=GREEN,zorder=11,bbox=dict(boxstyle='round,pad=.2',fc='white',ec='none'))
    # One metre in the same metric axes; keep it separate from the goal-radius marker.
    sx,sy=-.8,-6.25
    ax.plot([sx,sx+1],[sy,sy],color='#202020',lw=4,solid_capstyle='butt',zorder=12)
    ax.plot([sx,sx],[sy-.08,sy+.08],color='#202020',lw=1.5,zorder=12)
    ax.plot([sx+1,sx+1],[sy-.08,sy+.08],color='#202020',lw=1.5,zorder=12)
    ax.text(sx+.5,sy+.18,'1 m',ha='center',va='bottom',fontweight='bold',fontsize=12,zorder=12)
    handles=[Line2D([0],[0],color=ORANGE,lw=3,marker='P',ms=8,label='ESCAPE-Nav (Ours)'),
             Line2D([0],[0],color=BLUE,lw=3,marker='X',ms=8,label='Direct-goal PixelNav'),
             Line2D([0],[0],color=ORANGE,lw=1.5,ls='--',label='Goal radius: 1 m')]
    ax.legend(handles=handles,loc='upper right',frameon=True,facecolor='white',
              edgecolor='#D3D3D3',framealpha=.98,fontsize=10.7,borderpad=.8,labelspacing=.65)
    ax.set_title('Goal 4: ESCAPE-Nav and Direct-goal PixelNav',fontsize=15,pad=14)
    fig.subplots_adjust(left=.025,right=.99,bottom=.065,top=.92)
    fig.text(.5,.018,'Recorded trajectories aligned at the marked start; wall map shown for context.',
             ha='center',fontsize=8.7,color='#606060')
    fig.canvas.draw()
    # Assert equal displayed metre lengths and a genuine 20-map-pixel scale bar.
    origin=ax.transData.transform((0,0))
    dx=np.linalg.norm(ax.transData.transform((1,0))-origin)
    dy=np.linalg.norm(ax.transData.transform((0,1))-origin)
    assert abs(dx-dy)<1e-8 and abs(1/meta['resolution']-20)<1e-8
    for ext in ('png','pdf','svg'):
        fig.savefig(out/(STEM+'.'+ext),dpi=300,facecolor='white',bbox_inches='tight',pad_inches=.12)
    plt.close(fig)
    # Preserve the five alternatives so the qualitative selection is reviewable.
    fig,axes=plt.subplots(2,3,figsize=(13,7.7))
    selection=[]
    for ax,r in zip(axes.flat,candidates):
        background(ax,image,meta,(-1.6,12.6,-5.4,4.2))
        ax.plot(ours['xy'][:,0],ours['xy'][:,1],color=ORANGE,lw=1.3,alpha=.65)
        ax.plot(r['xy'][:,0],r['xy'][:,1],color=BLUE,lw=2)
        ax.scatter(*r['xy'][-1],s=50,marker='X',color=BLUE,zorder=5)
        ax.scatter(0,0,s=35,color=GREEN);ax.scatter(*goal,s=65,marker='*',color=ORANGE,zorder=5)
        ax.add_patch(Circle(goal,1,fill=False,color=ORANGE,lw=.8,ls='--'))
        selected=r['run']==PIX
        ax.set_title(r['run']+(' [selected]' if selected else ''),fontsize=9)
        ax.text(.02,.02,f"Final distance: {r['result']['goal_distance_m']:.2f} m",transform=ax.transAxes,
                fontsize=8,bbox=dict(fc='white',ec='none',alpha=.9))
        selection.append(dict(run=r['run'],selected=selected,
            final_goal_distance_m=r['result']['goal_distance_m'],
            duration_s=r['metrics']['elapsed_to_stop_s'],
            path_length_10hz_m=r['metrics']['path_length']['10hz_m'],
            field_report=r['field']))
    axes.flat[-1].axis('off')
    axes.flat[-1].text(.03,.7,'Selection: forward travel followed by a right turn.\n003 has the closest final distance of five runs.\nThis is a qualitative example, not a cohort score.',
        transform=axes.flat[-1].transAxes,fontsize=10,va='top',linespacing=1.6)
    fig.tight_layout();fig.savefig(details/'pixnav_candidates.png',dpi=180,bbox_inches='tight');plt.close(fig)
    for r,name in [(ours,'ours_trajectory.csv'),(pix,'pixnav_trajectory.csv')]:
        (details/name).write_bytes(read(r['root']/'trajectory.csv'))
    report=dict(ours_run=OURS,pixnav_run=PIX,goal_label='4',goal_labels_displayed=['4'],goal_xy_m=goal.tolist(),
        map_sha256=ours['binding']['map_sha256'],scale_bar_m=1.0,scale_bar_map_pixels=20,
        equal_axis_scale=True,limits_m=list(limits),
        frame='episode_start / fixed_start; x forward and y left',
        map_overlay='Identity overlay and pixel registration of the requested 0913 template; no new ICP registration.',
        geometry_modification='None: all exported pre-stop odom XY samples plotted in order; no smoothing, warping, or wall edits.',
        goal_heading_controlled=False,independent_ground_truth=False,paper_spl=None,
        selection_reason='User-selected right-turn example: 003 advances farther before turning right and has the closest final goal distance of all five recorded runs.',
        candidates=selection,
        selected_metrics={r['run']:dict(final_goal_distance_m=r['result']['goal_distance_m'],
            result_reason=r['result']['reason'],duration_s=r['metrics']['elapsed_to_stop_s'],
            path_length_10hz_m=r['metrics']['path_length']['10hz_m'],plotted_samples=len(r['xy']),
            plotted_endpoint_xy_m=r['xy'][-1].tolist(),field_report=r['field'],
            navigation_image=r['metrics']['images']['escape']) for r in (ours,pix)},
        caveats=['Ours arrival is the logged 1 m odometry criterion; physical arrival is unverified in its field report.',
                 'The selected PixelNav run has unknown intervention/contact/obstruction status.',
                 'The two runs use different navigation-image revisions and are selected qualitative examples.'])
    (details/'figure_metadata.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    (details/'input_provenance.json').write_text(json.dumps(INPUTS,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(png=str(out/(STEM+'.png')),ours=OURS,pixnav=PIX,
        source_files_verified=sum(v['matches_prior_evidence_manifest'] for v in INPUTS.values()),
        scale_m=1.0,samples={OURS:len(ours['xy']),PIX:len(pix['xy'])}),ensure_ascii=False))


if __name__=='__main__':
    main()
