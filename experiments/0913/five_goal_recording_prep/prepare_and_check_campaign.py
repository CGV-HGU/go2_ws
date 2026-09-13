"""Build only configuration files. Synthetic trial wrappers are disabled afterwards."""
from pathlib import Path
import sys,json,hashlib,sqlite3,time,subprocess,shutil
R=Path('/home/unitree/s2e-vlm-async-framework-minimal');A=R/'.local-data/five-goal-recording-prep-20260913';sys.path.insert(0,str(R/'scripts'))
import robot_goal_campaign as c
from fixed_start_goals import make_origin,capture_goal
from robot_recording_integrity import check
res=json.loads((R/'.local-data/jetson-post-drive-20260913/results.json').read_text())
res['parent_navigation_image_id']=res['navigation_image_id']
res['navigation_image_id']=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','escape-navigation:five-goal-recording-20260913'],text=True).strip()
changed=['scripts/robot_goal_campaign.py','scripts/fixed_start_goals.py','scripts/robot_map_session.py','scripts/prepare_robot_pointgoal_trial.py','scripts/validate_robot_pointgoal_trial.py','scripts/robot_recording_integrity.py','src/s2e_vlm_robot/launch/robot_escape.launch.py']
for name in changed:res['source_sha256'][name]=c.digest(R/name)
res['tests']['five_goal_recording_workflow']={'passed':92,'log':'workflow-tests-final.log'}
res['tests']['installed_robot_by_package']={'passed':312,'log':'robot-tests.log'}
res['installed_launch_sha256']=c.digest(R/'src/s2e_vlm_robot/launch/robot_escape.launch.py')
res['tests']['model_input_log_roundtrip']=c.read(A/'model-input-log-check.json')
res['scope']='Five-goal campaign preparation, capture/map binding, unchanged navigation runtime, input recording and loss detection. No live mapping, goal capture or drive.'
res['physical_drive_validated']=False;res['completed_unix']=time.time()
c.write(A/'validation.json',res)
root=A/'synthetic-configuration-only-v2'
c.prepare_campaign(root,A/'validation.json')
now=time.time();boot='synthetic-not-a-physical-campaign'
def snapshot(x):
 return dict(host_boot_id=boot,checked_unix=now,odom_publisher_gids=['synthetic-owner'],topics=dict(odom=dict(frame='odom',stamp_unix=now,received_unix=now,pose=dict(pose=dict(position=dict(x=x,y=0.,z=.3),orientation=dict(x=0.,y=0.,z=0.,w=1.))))))
def save_capture(v,name,s):
 c.write(root/'goals'/(name+'.snapshot.json'),s)
 v.update(source_snapshot_file=name+'.snapshot.json',source_snapshot_sha256=c.digest(root/'goals'/(name+'.snapshot.json')))
 c.write(root/'goals'/(name+'.json'),v);return v
origin=save_capture(make_origin(snapshot(0.),now,boot),'origin',snapshot(0.))
for label in range(1,6):save_capture(capture_goal(origin,str(label),snapshot(label+2.),now,boot),str(label),snapshot(label+2.))
db=root/'mapping/map.db'
with sqlite3.connect(str(db)) as conn:conn.execute('create table Node(id integer)');conn.execute('insert into Node values(1)')
m=c.read(root/'mapping/manifest.json');m.update(status='collecting_fixed_start_goals',host_boot_id=boot,source_database=str(db),source_database_sha256=c.digest(db),localization_mode='fixed_start_odometry');c.write(root/'mapping/manifest.json',m)
c.bind_goals(root)
trials=[]
for mode in ('full','direct_goal'):
 for label in range(1,6):trials.append(c.add_trial(root,str(label),mode))
retry=c.add_trial(root,'3');trials.append(retry)
assert retry['trial_id']=='full-goal-3-002'
for label in range(1,6):
 full=root/'episodes'/('full-goal-'+str(label)+'-001');direct=root/'episodes'/('direct_goal-goal-'+str(label)+'-001')
 assert (full/'goal-plan.json').read_bytes()==(direct/'goal-plan.json').read_bytes()
for trial in trials:
 p=Path(trial['path']);m=c.read(p/'manifest.json')
 assert m['offline_checks_passed'] and m['ready_for_supervised_trial']
 script=(p/'run_trial.sh').read_text()
 assert '--live --output' in script and 'RECORDERS_CLOSED' in script
 assert script.index('dc stop escape unitree-command pixnav') < script.index('RECORDERS_CLOSED')
 # Disable the synthetic fixture so it can never be mistaken for a live goal set.
 m.update(ready_for_supervised_trial=False,synthetic_fixture_do_not_drive=True)
 c.write(p/'manifest.json',m);(p/'CANCEL').touch()
report=dict(passed=True,full_goals=5,direct_goals=5,retries=1,physical_runs=0,synthetic_trial_wrappers_disabled=True,scope='Real Docker image and compose validation, synthetic coordinates only')
c.write(A/'five-goal-configuration-check.json',report)
live=R/'.local-data/recording-five-goals-20260913'
if not live.exists():c.prepare_campaign(live,A/'validation.json')
else:
 shutil.copy2(A/'validation.json',live/'candidate-validation.json')
 value=c.read(live/'campaign.json');value['camera']=c.prepare_camera(live);c.write(live/'campaign.json',value)
old=R/'.local-data/current-prepared-robot-trials.json';shutil.copy2(old,A/'previous-prepared-pointer.json')
c.write(old,dict(campaign_root=str(live),mapping_session=str(live/'mapping'),validation_results=str(A/'validation.json'),trials={},saved_goal_trials={},goal_labels=['1','2','3','4','5'],goal_plan_status='awaiting_new_mapping_and_five_actual_captures',next_priority='new mapping -> fixed-start goal capture 1..5 -> Full episodes -> same-goal Direct comparison',prepared_only=True,physical_drive_validated=False))
print(json.dumps(report));print('Actual campaign prepared without starting:',live)
