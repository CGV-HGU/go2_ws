#!/usr/bin/env python3
"""Reproducible descriptive figures and timing audit from exported evidence."""
import csv
import gzip
import json
import math
from pathlib import Path
from datetime import datetime,timezone,timedelta
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
DATA=Path('/home/unitree/s2e-vlm-async-framework-minimal/.local-data')
FIG=OUT/'figures'; FIG.mkdir(exist_ok=True)
KST=timezone(timedelta(hours=9))
with gzip.open(OUT/'plot_data.json.gz','rt') as f: runs=json.load(f)


def save(fig,name):
    fig.savefig(FIG/(name+'.png'),dpi=160,bbox_inches='tight')
    fig.savefig(FIG/(name+'.pdf'),bbox_inches='tight')
    plt.close(fig)


def serial(p,obj):
    p.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


keys=['original5/full-goal-4-009','recaptured5/full-goal-4-002','recaptured5/full-goal-4-003']
labels=['Earlier Ours: radius reached','Later Ours: stale control pose','Latest Ours: network stall + manual relocation']
colors=['tab:green','tab:orange','tab:red']
fig,axes=plt.subplots(1,2,figsize=(12,5))
for key,label,color in zip(keys,labels,colors):
    r=runs[key]; x=[v['x_m'] for v in r['odom']];y=[v['y_m'] for v in r['odom']]
    if key!=keys[2]:axes[0].plot(x,y,label=label,c=color,lw=2.2)
    axes[1].plot([v['t_s'] for v in r['pg']],[v['distance_m'] for v in r['pg']],label=label,c=color)
for j,key in enumerate(sorted(k for k in runs if k.startswith('recaptured5/direct_goal') and runs[k]['metric']['started'])):
    r=runs[key];axes[0].plot([v['x_m'] for v in r['odom']],[v['y_m'] for v in r['odom']],lw=1,alpha=.9,ls='--',c=plt.cm.cool(.1+j*.18),label='Direct '+key[-3:])
goal=runs[keys[0]]['goal'];axes[0].plot(*goal,'r*',ms=13);axes[0].add_patch(plt.Circle(goal,1,fill=False,ls='--',color='red'))
axes[0].plot(0,0,'ko');axes[0].axis('equal');axes[0].set(xlabel='Forward (m)',ylabel='Left (m)',title='Goal 4: estimates in each marked-start frame')
axes[1].axhline(1,c='grey',ls='--');axes[1].set(xlabel='Time since goal publication (s)',ylabel='Estimated distance (m)',title='Manual relocation is retained in the red trace')
for ax in axes:ax.legend(fontsize=7);ax.grid(alpha=.2)
fig.suptitle('Descriptive comparison; start placement / odometry accuracy not independently surveyed',fontsize=10)
fig.tight_layout(rect=(0,0,1,.92));save(fig,'goal4_comparison')

# Sensor freshness is evaluated at the controller, not by the logging subscriber.
r=runs[keys[1]]
fault=next(v for v in r['motion'] if v.get('status')=='LOCALIZED_POSE_STALE')['log_unix_s']
audit={'fault_unix_s':fault,'fault_kst':datetime.fromtimestamp(fault,KST).isoformat(),'threshold_s':.5}
fig,ax=plt.subplots(figsize=(9,3.8))
for name,ls in [('odom','-'),('ctrl','-')]:
    p=r[name]
    pre=[v for v in p if datetime.fromisoformat(v['utc']).timestamp()<=fault]
    latest=max(pre,key=lambda v:v['source_stamp_s'])
    audit[name+'_latest_at_recorder']={'source_age_at_fault_s':fault-latest['source_stamp_s'],'receipt_before_fault_s':fault-datetime.fromisoformat(latest['utc']).timestamp()}
    win=[v for v in p if abs(datetime.fromisoformat(v['utc']).timestamp()-fault)<2]
    ax.plot([datetime.fromisoformat(v['utc']).timestamp()-fault for v in win],[v['receipt_age_s'] for v in win],'.-',label=name+' at recorder')
points=[v for v in r['motion'] if v.get('odom_stamp_ns') and abs(v['log_unix_s']-fault)<2]
ax.plot([v['log_unix_s']-fault for v in points],[v['log_unix_s']-v['odom_stamp_ns']/1e9 for v in points],'.-',label='Controller odom at command log')
ax.axhline(.5,c='red',ls='--',label='Controller pose freshness limit');ax.axvline(0,c='black',ls=':')
ax.set(xlabel='Time relative to stale fault (s)',ylabel='Recorded data age (s)',title='Latest control pose inside controller was not logged')
ax.grid(alpha=.2);ax.legend(fontsize=8);fig.tight_layout();save(fig,'stale_pose_timing')
audit['interpretation']='Fresh data reached the independent recorder. Controller receive/processing delay is suspected, but its retained pose stamp and callback timing were not recorded; root cause is not proven.'
serial(OUT/'stale_pose_audit.json',audit)

r=runs[keys[2]]
net=json.loads((OUT/'network/events.json').read_text())
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
axes[0].plot([v['t_s'] for v in r['pg']],[v['distance_m'] for v in r['pg']],color='tab:red')
axes[0].set(ylabel='Goal distance (m)',title='Latest Ours 4: API timeouts, then operator relocation (exact start unlogged)')
types={'navigation':0,'candidate_screener':1,'point_verifier':2}
intervals=[]
for c in r['calls']:
    end=c['approximate_finish_t_s'];begin=c['approximate_begin_t_s']
    color='tab:red' if c['error_code'] else 'tab:green'
    axes[1].plot([begin,end],[types[c['call_type']]]*2,c=color,alpha=.6,lw=4)
    if c['error_code']:intervals.append((begin,end))
axes[1].set(yticks=[0,1,2],yticklabels=list(types),ylabel='HTTP call type')
for e in net:
    t=e['unix_s']-r['start_unix']
    if 0<=t<=r['metric']['duration_s'] and ('CTRL-EVENT-DISCONNECTED' in e['text'] or 'USB disconnect' in e['text'] or 'link timed out' in e['text']):
        axes[2].vlines(t,0,1,color='tab:purple' if e['kind']=='usb' else 'tab:blue')
        axes[2].text(t,1.02,'USB' if e['kind']=='usb' else 'Wi-Fi',rotation=90,fontsize=7)
axes[2].set(ylim=(0,1.6),yticks=[],xlabel='Time since goal publication (s)',ylabel='Link events')
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();save(fig,'network_stall_timeline')
merged=[]
for a,b in sorted(intervals):
    if merged and a<=merged[-1][1]:merged[-1][1]=max(b,merged[-1][1])
    else:merged.append([a,b])
serial(OUT/'network_stall_audit.json',{'failed_http_requests':len(intervals),'timeout_duration_sum_s':sum(b-a for a,b in intervals),
       'approximate_union_of_failed_request_intervals_s':sum(b-a for a,b in merged),'interval_time_precision_note':'Completion timestamps are whole UTC seconds; starts reconstructed by subtracting measured latency. Parallel requests overlap; summed durations are not stopped duration.',
       'manual_relocation_confirmed':True,'manual_relocation_start_time_known':False,
       'failure_sequence':'First request timeout completed 00:57:43; Wi-Fi disconnect logged 00:57:50; link failure at 00:58:06; parallel verification timeouts continued through 01:00:15; operator relocated robot before ODOMETRY_JUMP termination at 01:01:29.',
       'causal_limit':'Network unavailability is directly corroborated for the later stall. First timeout predates the logged link-down event; exact onset and pure server latency are not established.'})

# Show the exact initial goal mask and the actual image on the final model step.
direct=sorted(k for k in runs if k.startswith('recaptured5/direct_goal') and runs[k]['metric']['started'])
fig,axes=plt.subplots(len(direct),3,figsize=(12,2.8*len(direct)))
facts=[]
for row,key in enumerate(direct):
    name=key.split('/')[1]
    root=OUT/'episodes/recaptured5'/name/'selected_policy_inputs'
    if root.exists():
        initial=root/'session_0001_reset.npz'
    else:
        root=DATA/'recording-five-goals-recaptured5-20260914/episodes'/name/'full5m/policy-inputs'
        initial=next(root.glob('*/session_0001_reset.npz'))
    steps=sorted(initial.parent.glob('session_0001_step_*.json'))
    last=steps[-1];meta=json.loads(last.read_text())
    with np.load(initial) as z:
        rgb=z['goal_image'][:,:,::-1];mask=z['goal_mask'];policy=z['policy_goal_image'][0];pmask=z['policy_goal_mask'][0,:,:,0]
    with np.load(last.with_suffix('.npz')) as z:stop=z['observation'][:,:,::-1]
    ys,xs=np.where(mask>0);py,px=np.where(pmask>0)
    axes[row,0].imshow(rgb);axes[row,0].plot(xs.mean(),ys.mean(),'o',ms=9,mfc='none',mec='red',mew=2)
    axes[row,1].imshow(policy);axes[row,1].plot(px.mean(),py.mean(),'o',ms=8,mfc='none',mec='red',mew=2)
    axes[row,2].imshow(stop)
    pred=meta['goal_prediction'];axes[row,2].plot(pred[1]*stop.shape[1],pred[0]*stop.shape[0],'o',ms=9,mfc='none',mec='yellow',mew=2)
    titles=[name+' initial RGB + goal mask','Actual 224x224 policy goal input',f'Model STOP; residual {runs[key]["metric"]["final_goal_distance_m"]:.2f} m']
    for ax,title in zip(axes[row],titles):ax.set_title(title,fontsize=9);ax.axis('off')
    facts.append(dict(run=name,policy_steps=runs[key]['metric']['policy_outputs'],final_action=meta['action'],stop_source=meta['stop_source'],
                      raw_logits=meta['raw_logits'],effective_logits=meta['effective_logits'],
                      distance_prediction_raw=meta['distance_prediction'],distance_prediction_units='not established',
                      initial_mask_centroid_uv=[float(xs.mean()),float(ys.mean())],
                      final_goal_prediction_vu=pred,physical_goal_distance_estimate_m=runs[key]['metric']['final_goal_distance_m']))
fig.suptitle('Direct PixelNav: red = fixed initial goal mask; yellow = model-predicted goal on STOP frame\nBGR to RGB display matches preprocessing. Visual markers are not depth/ground truth.',fontsize=11)
fig.tight_layout(rect=(0,0,1,.96));save(fig,'direct_goal_inputs_and_stop')
serial(OUT/'direct_goal_stop_audit.json',facts)
print('Created review figures and audits')
