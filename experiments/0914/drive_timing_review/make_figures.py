import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
data=json.loads((OUT/'results.json').read_text())
names=['original5/full-goal-4-009','long5/full-goal-5-001','recaptured5/full-goal-5-001','recaptured5/direct_goal-goal-4-002']
labels=['Ours goal 4: reached','Ours old goal 5: 360s limit','Ours new goal 5: rotation timeout','Direct goal 4: model STOP']
fig,ax=plt.subplots(figsize=(11,4.8))
rows=[next(x for x in data['episodes'] if x['run']==name) for name in names]
left=[0.]*len(rows)
for fields,label,color in [(['forward_s','look_forward_20cm_s'],'Forward macros incl. braking','#3b8bc2'),
                          (['policy_turn_s'],'Policy turns incl. settling','#e69f00'),
                          (['observation_or_alignment_turn_s'],'Observation / alignment turns incl. settling','#9a71bd'),
                          (['other_time_s'],'Outside motion macros','#b9b9b9')]:
    values=[sum(r.get(k,0) for k in fields) for r in rows]
    ax.barh(range(len(rows)),values,left=left,color=color,label=label)
    for i,v in enumerate(values):
        if v>=20:ax.text(left[i]+v/2,i,f'{v:.0f}s',ha='center',va='center',fontsize=9)
    left=[a+b for a,b in zip(left,values)]
ax.set_yticks(range(len(rows)));ax.set_yticklabels(labels);ax.invert_yaxis()
ax.set_xlabel('Seconds from goal dispatch to first stop request')
ax.set_title('Recorded time allocation: examples, not matched-goal success-rate comparison')
ax.legend(loc='upper center',bbox_to_anchor=(.4,-.15),ncol=2,fontsize=8)
ax.grid(axis='x',alpha=.2);fig.tight_layout()
for extension in ['png','pdf']:fig.savefig(OUT/('time_allocation.'+extension),dpi=170,bbox_inches='tight')
plt.close(fig)

motion=json.loads((OUT.parent/'night_log_review/episodes/recaptured5/full-goal-5-001/motion_events.json').read_text())
term=motion[-1];start=term['log_unix_s']-term['elapsed_s']
ticks=[e for e in motion if e['event']=='rotate_command' and e['log_unix_s']>start]
import math
fig,axes=plt.subplots(2,1,figsize=(10,5.5),sharex=True)
x=[t['elapsed_s'] for t in ticks]
error=[math.degrees(t['measured_progress']-t['target']) for t in ticks]
axes[0].plot(x,error,lw=1,label='Recorded yaw error')
axes[0].axhspan(-3,3,color='green',alpha=.12,label='Requested +/-3 degree band')
axes[0].axvline(2.252889534,color='gray',ls='--',label='Angular command becomes zero')
axes[0].set_ylim(-8,5);axes[0].set_ylabel('Error (degrees)');axes[0].legend(fontsize=8)
axes[1].plot(x,[t['angular_command_radps'] for t in ticks],label='Commanded yaw rate')
axes[1].plot(x,[t['measured_yaw_rate_radps'] for t in ticks],alpha=.7,label='Controller-reported yaw rate')
axes[1].axhline(.1,ls=':',color='gray');axes[1].axhline(-.1,ls=':',color='gray')
axes[1].set_xlabel('Seconds from final rotation start');axes[1].set_ylabel('Yaw rate (rad/s)');axes[1].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('New goal 5: angle reached; zero command persists until motion timeout')
fig.tight_layout(rect=(0,0,1,.95))
for extension in ['png','pdf']:fig.savefig(OUT/('goal5_rotation.'+extension),dpi=170,bbox_inches='tight')
plt.close(fig)
