"""Inspect all ten prepared profiles and exercise frozen recorder stop expressions."""
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

OUT = Path(__file__).resolve().parent
REPO = Path('/home/unitree/s2e-vlm-async-framework-minimal')
ROOT = REPO/'.local-data/recording-five-goals-recaptured5-20260914'
sys.path.insert(0,str(REPO/'scripts'))
from run_saved_robot_goal import runner_timeout_s, required_recording_space_bytes
from validate_robot_pointgoal_trial import validate_timing, validate_recorder_timing


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    c=json.loads((ROOT/'campaign.json').read_text())
    before=json.loads((OUT/'before/campaign.json').read_text())
    assert c['goal_plan_sha256']==before['goal_plan_sha256']==digest(ROOT/'goal-plan.json')
    assert c['map_sha256']==before['map_sha256']==digest(Path(c['map_database']))
    assert c['navigation_image']==before['navigation_image']
    profiles=[]
    for method in ('full','direct_goal'):
        for label in map(str,range(1,6)):
            entry=next(e for e in reversed(c['trials']) if e['method']==method and e['goal_label']==label and e['status']=='prepared')
            p=Path(entry['path']);m=json.loads((p/'manifest.json').read_text());cond=m['runtime_conditions']
            assert cond['max_duration_s']==1800 and cond['recorder_max_duration_s']==2040
            assert cond['navigation']==method and cond['goal_label']==label
            assert m['offline_checks_passed'] and not m['physical_drive_validated']
            assert runner_timeout_s(p)==1980
            assert not any((p/name).exists() for name in ('USED','START_GOAL','STACK_STARTED'))
            validate_timing(cond,(p/'run_trial.sh').read_text(),c['trial_settings'])
            validate_recorder_timing(p,cond,(p/'run_trial.sh').read_text())
            for filename,sha in m['sha256'].items():assert digest(p/filename)==sha
            assert digest(p/'goal-plan.json')==c['goal_plan_sha256']
            profiles.append({'trial_id':entry['trial_id'],'path':str(p),'mode':method,'goal':label,
                             'max_duration_s':1800,'shell_timeout_s':1860,'runner_timeout_s':1980,
                             'recorder_timeout_s':2040,'manifest_sha256':digest(p/'manifest.json')})
    # Evaluate the actual prepared loops, without importing ROS or starting a node.
    frozen=Path(profiles[0]['path'])
    loop_checks=[]
    with tempfile.TemporaryDirectory() as tmp:
        temp=Path(tmp)
        for filename in ('full5m/record_body.py','full5m/observe_camera_and_sweep.py','record_camera_timing.py'):
            tree=ast.parse((frozen/filename).read_text())
            candidates=[n for n in ast.walk(tree) if isinstance(n,ast.While) and any(isinstance(x,ast.Name) and x.id in ('stop','stopped') for x in ast.walk(n.test))]
            assert len(candidates)==1
            expression=compile(ast.Expression(candidates[0].test),filename,'eval')
            env={'stop':False,'stopped':False,'start':0,'deadline':2040,'root':temp,'p':temp,'args':SimpleNamespace(output=temp)}
            observed={}
            for t in (600,1800,1980,2040):
                env['time']=SimpleNamespace(monotonic=lambda:t)
                observed[str(t)]=eval(expression,{},env)
                assert observed[str(t)] is (t<2040)
            env['time']=SimpleNamespace(monotonic=lambda:600)
            (temp/'STOP').touch();assert eval(expression,{},env) is False;(temp/'STOP').unlink()
            loop_checks.append({'file':filename,'continue_at_elapsed_s':observed,'stop_marker_still_ends_loop':True})
    # The prior raw evidence manifests are independent of the changed preparation scripts.
    preserved=[]
    for entry in before['trials']:
        path=Path(entry['path'])/'manifest.json'
        if path.exists():
            old=json.loads(path.read_text())
            assert old['runtime_conditions']['max_duration_s']==360
            preserved.append({'trial_id':entry['trial_id'],'max_duration_s':360,'manifest_sha256':digest(path)})
    free=shutil.disk_usage(ROOT).free
    report={'checked_at':datetime.now(timezone.utc).isoformat(),'profiles':profiles,'recorder_loop_checks':loop_checks,
            'older_profiles_retained':preserved,'shared_map_goal_and_navigation_image_unchanged':True,
            'free_bytes':free,'required_start_free_bytes':required_recording_space_bytes(1800),
            'one_long_episode_space_check_passed':free>=required_recording_space_bytes(1800),
            'motion_requested':False,'real_1800_second_recording_verified':False,
            'scope':'File/argument/hash validation and frozen loop time/STOP expressions; not real hardware recording or goal success.'}
    (OUT/'prepared-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'profiles':len(profiles),'recorders':len(loop_checks),'older_profiles':len(preserved),
                      'free_GiB':free/1024**3,'required_GiB':required_recording_space_bytes(1800)/1024**3},indent=2))


if __name__=='__main__':main()
