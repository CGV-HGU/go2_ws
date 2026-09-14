#!/usr/bin/env python3
"""Apply separately confirmed field facts and verify the exported evidence."""
import csv
import gzip
import hashlib
import importlib.util
import json
import math
import re
import shutil
from pathlib import Path

OUT=Path(__file__).resolve().parent
DATA=Path('/home/unitree/s2e-vlm-async-framework-minimal/.local-data')


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def write_json(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


inventory={x['exported']:x for x in json.loads((OUT/'evidence_inventory.json').read_text())}
campaign=DATA/'recording-five-goals-recaptured5-20260914'
for name in ['direct_goal-goal-4-00'+str(i) for i in range(2,7)]:
    reset=next((campaign/'episodes'/name/'full5m/policy-inputs').glob('*/session_0001_reset.npz'))
    last=sorted(reset.parent.glob('session_0001_step_*.json'))[-1]
    for src in [reset,reset.with_suffix('.json'),last,last.with_suffix('.npz')]:
        dest=OUT/'episodes/recaptured5'/name/'selected_policy_inputs'/src.name
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
        rel=str(dest.relative_to(OUT));h=sha(src)
        inventory[rel]=dict(source=str(src),exported=rel,source_bytes=src.stat().st_size,source_sha256=h,
                            exported_bytes=dest.stat().st_size,exported_sha256=h,encoding='identity')
for src in (campaign/'quick-results/direct-goal-4-002-evaluation').glob('*'):
    if src.is_file():
        dest=OUT/'episodes/recaptured5/direct_goal-goal-4-002/previous_review'/src.name
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
        rel=str(dest.relative_to(OUT));h=sha(src)
        inventory[rel]=dict(source=str(src),exported=rel,source_bytes=src.stat().st_size,source_sha256=h,
                            exported_bytes=dest.stat().st_size,exported_sha256=h,encoding='identity')
write_json(OUT/'evidence_inventory.json',list(inventory.values()))

metrics=json.loads((OUT/'episodes_summary.json').read_text())
facts=json.loads((OUT/'operator_clarifications.json').read_text())
confirm={(d['campaign'],d['run']):d for d in facts['confirmations']}
for m in metrics:
    field=confirm.get((m['campaign'],m['run']),{})
    intervention=field.get('direct_intervention')
    m['operator_intervention_reviewed']=intervention
    m['autonomous_path_usable']=m['odom_path_10hz_m'] is not None and intervention is False
    m['autonomous_path_use_status']='accepted_relative_estimate' if m['autonomous_path_usable'] else ('manual_intervention_not_an_autonomous_length' if intervention else 'field_context_unverified')
    m['provisional_protocol_success']=None
    if not m['started']:
        m['metric_use']='prestart_excluded_from_driving_denominator'
    elif m['campaign']=='recaptured5' and m['run']=='full-goal-4-003':
        m['provisional_protocol_success']=0
        m['metric_use']='network_stall_then_confirmed_manual_relocation; autonomous_path_invalid; intervention_failure_for_end_to_end_protocol'
    elif m['campaign']=='recaptured5' and m['run']=='direct_goal-goal-4-002':
        m['provisional_protocol_success']=0
        m['metric_use']='confirmed_unassisted_early_STOP_failure_under_declared_fixed_start_protocol; final_paper_cohort_pending'
    elif m['campaign']=='original5' and m['run']=='full-goal-5-006':
        m['provisional_protocol_success']=0
        m['metric_use']='operator_intervened_before_wall_contact; whole_run_path_not_certified_autonomous'
    else:
        m['metric_use']='descriptive_recording; field_or_physical_arrival_review_pending; shortest_path_reference_missing'
write_json(OUT/'episodes_summary.json',metrics)
with (OUT/'episodes_summary.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(metrics[0]));writer.writeheader();writer.writerows(metrics)
with gzip.open(OUT/'plot_data.json.gz','rt') as f:plot=json.load(f)
for m in metrics:plot[m['campaign']+'/'+m['run']]['metric']=m
with (OUT/'plot_data.json.gz').open('wb') as f:
    with gzip.GzipFile(filename='',fileobj=f,mode='wb',mtime=0,compresslevel=6) as gz:gz.write(json.dumps(plot,ensure_ascii=False,allow_nan=False).encode())
spec=importlib.util.spec_from_file_location('export_review',OUT/'export_review.py')
exp=importlib.util.module_from_spec(spec);spec.loader.exec_module(exp)
key='recaptured5/full-goal-4-003';d=plot[key]
exp.figure_episode(OUT/'episodes'/key,key,d['odom'],d['pg'],d['goal'],d['metric'])

issues=[];verified=0;historic_matches=0;historic_mismatches=[]
old_manifest_cache={}
for item in inventory.values():
    src=Path(item['source']);dest=OUT/item['exported']
    if sha(src)!=item['source_sha256']:issues.append('source changed: '+str(src))
    if sha(dest)!=item['exported_sha256']:issues.append('export hash mismatch: '+str(dest))
    h=hashlib.sha256()
    opener=gzip.open if item['encoding']=='gzip' else open
    with opener(dest,'rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    if h.hexdigest()!=item['source_sha256']:issues.append('decoded bytes mismatch: '+str(dest))
    verified+=1
    # Compare copied episode raw files with the previously frozen raw manifest.
    if '/episodes/' in str(src):
        parts=src.parts;ix=parts.index('episodes');run_root=Path(*parts[:ix+2]);rel='/'.join(parts[ix+2:])
        arch=run_root.parent.parent/'archives'/run_root.name/'raw-manifest.json'
        if arch.exists():
            if str(arch) not in old_manifest_cache:
                old_manifest_cache[str(arch)]={r['path']:r['sha256'] for r in json.loads(arch.read_text())['files']}
            expected=old_manifest_cache[str(arch)].get(rel)
            if expected==item['source_sha256']:historic_matches+=1
            elif expected:historic_mismatches.append(dict(source=str(src),archived_sha256=expected,current_sha256=item['source_sha256']))

metric_checks=0
for m in metrics:
    p=OUT/'episodes'/m['campaign']/m['run']
    archived=p/'original_archive/episode.json';current=json.loads((p/'episode.recomputed.json').read_text())
    if archived.exists():
        old=json.loads(archived.read_text())
        if old['result_reason']!=current['result_reason']:issues.append('changed archived outcome: '+str(p))
        for k in ['elapsed_to_stop_s','minimum_recorded_goal_distance_m']:
            if old.get(k) is not None and abs(old[k]-current[k])>1e-6:issues.append('metric mismatch '+k+': '+str(p))
        for k,v in (old.get('path_length') or {}).items():
            if k.endswith('hz_m') and abs(v-current['path_length'][k])>1e-6:issues.append('path mismatch '+k+': '+str(p))
    if m['started']:
        with (p/'trajectory.csv').open() as f:poses=list(csv.DictReader(f))
        if len(poses)<2:issues.append('missing trajectory '+str(p))
        if any(float(b['t_s'])<=float(a['t_s']) for a,b in zip(poses,poses[1:])):issues.append('nonmonotonic CSV '+str(p))
        if any(not math.isfinite(float(r[k])) for r in poses for k in ['x_m','y_m','yaw_rad','t_s']):issues.append('nonfinite CSV '+str(p))
    metric_checks+=1
direct=json.loads((OUT/'direct_goal_stop_audit.json').read_text())
for row in direct:
    if row['final_action']!=0 or row['stop_source']!='model' or row['raw_logits']!=row['effective_logits'] or max(range(6),key=lambda i:row['raw_logits'][i])!=0:
        issues.append('direct STOP evidence mismatch '+row['run'])
if any(m['paper_spl'] is not None for m in metrics):issues.append('unreviewed SPL published')
if plot[key]['metric']['autonomous_path_usable'] is not False:issues.append('manual relocation mislabeled')
files=[p for p in OUT.rglob('*') if p.is_file()]
too_large=[str(p.relative_to(OUT)) for p in files if p.stat().st_size>=100*1024*1024]
if too_large:issues.append('files >=100MiB: '+str(too_large))
summary=dict(passed=not issues,issues=issues,exported_evidence_files_verified=verified,
             original_manifest_hash_matches=historic_matches,original_manifest_hash_mismatches=historic_mismatches,
             episode_metric_checks=metric_checks,started_episodes=sum(m['started'] for m in metrics),
             prestart_or_incomplete=sum(not m['started'] for m in metrics),direct_model_STOP_checks=len(direct),
             manually_relocated_run_excluded_from_autonomous_path=True,final_paper_SPL_unset=True,
             bundle_bytes=sum(p.stat().st_size for p in files),largest_file_bytes=max(p.stat().st_size for p in files),
             scope='Copied evidence hashes and gzip payloads checked; existing metric calculation independently matched archived values. Entire large rosbag corpus not reread; runtime/hardware not retested.')
if historic_mismatches:
    summary['passed']=False;summary['issues'].append('Historical raw-manifest mismatches require explicit review')
write_json(OUT/'validation.json',summary)
print(json.dumps(summary,ensure_ascii=False))
