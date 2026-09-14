"""Run installed navigation code tests in an isolated container, grouped by package."""
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys

OUT=Path('/out')
assert {p.name for p in Path('/sys/class/net').iterdir()}=={'lo'}
modules=['s2e_vlm_robot.motion','s2e_vlm_robot.action_executor_node',
         's2e_vlm_robot.sport_client','s2e_vlm_nodes.runtime.pixnav',
         's2e_vlm_nodes.runtime.vlm','s2e_vlm_nodes.runtime.sweep_scheduler']
imports={}
for name in modules:
    module=importlib.import_module(name);p=Path(module.__file__)
    assert str(p).startswith('/opt/s2e-robot-minimal/'),str(p)
    host=Path('/verification/src')/name.split('.')[0]/Path(*name.split('.')).with_suffix('.py')
    imports[name]=dict(installed=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                      matches_host=host.is_file() and host.read_bytes()==p.read_bytes())
groups={
 'profile-check':['/verification/src/s2e_vlm_robot/test/test_full_profile.py::test_full_profile_preserves_every_maintained_policy_method'],
 'async-boundary-tests':['/verification/src/s2e_vlm_nodes/test/'+name+'.py' for name in [
  'test_robot_look_forward','test_robot_five_item_faults','test_robot_physical_async_handoff',
  'test_robot_action_failure','test_robot_action_pose_ordering','test_robot_async_correction_handoff',
  'test_robot_camera_failure_feedback','test_robot_queued_goal_lifetime',
  'test_robot_sweep_rotation_failure','test_robot_observation_feedback_fence',
  'test_robot_terminal_feedback_lifetime','test_robot_preturn_observation_wait',
  'test_robot_goal_application','test_robot_goal_candidate_ranking',
  'test_robot_direct_goal_graph','test_robot_five_map_goals',
  'test_robot_vlm_http_transport','test_robot_sweep_return_field']],
}
results={}
for group,paths in groups.items():
    with (OUT/(group+'.log')).open('w') as f:
        command=[sys.executable,'-m','pytest','-q','-p','no:cacheprovider','--junitxml=/out/'+group+'.xml']+paths
        result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=300)
    results[group]=dict(returncode=result.returncode,command=command)
(OUT/'boundary-checks.json').write_text(json.dumps(dict(imports=imports,groups=results,
 physical_robot_transport=False,network='none',method='Test fixtures substitute recording/mocked transport; real ROS message and callback code where applicable.'),indent=2)+'\n')
print(json.dumps(results,indent=2))
sys.exit(int(any(r['returncode'] for r in results.values())))
