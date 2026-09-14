"""Installed Direct runtime session lifetime, with fresh synthetic input and fake policy."""
import json
import os
from pathlib import Path
import sys
from unittest.mock import patch

import rclpy
from s2e_vlm_nodes.runtime.pixnav import PixNavRuntimeNode,PixNavRuntimeError
from s2e_vlm_nodes.pixnav_runtime_node import NODE_CONTRACT
from s2e_vlm_core.pose_buffer import Pose2D

sys.path.insert(0,'/verification/src/s2e_vlm_nodes')
from test.test_pixnav_runtime_node import SequenceWorker,identity,frame,decision

assert {p.name for p in Path('/sys/class/net').iterdir()}=={'lo'}
env={'S2E_RUNTIME_ROLE':'robot_full','S2E_SENSOR_PROFILE':'robot',
     'S2E_SENSOR_CONFIG_DIR':'/work/config/sensors','PIXNAV_PHYSICAL_BODY_LOOK':'true',
     'PIXNAV_FIXED_CAMERA_NOOP':'false','PIXNAV_FIXED_CAMERA_VIEW_ON_LOOK':'false',
     'PIXNAV_FIXED_CAMERA_VIEW_ZOOM':'1','PIXNAV_LOOK_EXECUTION':'forward_0p2',
     'PIXNAV_EXECUTION_MODE':'async_true','PIXNAV_MAX_GO_STEPS':'500'}
rows=[]
for seconds in [299.,301.,1799.]:
    with patch.dict(os.environ,env):
        rclpy.init();worker=SequenceWorker(('move_forward',));worker.backend_identity=identity()
        node=PixNavRuntimeNode(NODE_CONTRACT,worker=worker)
    try:
        r=node.runtime
        r.update_apply_pose(Pose2D(1.,2.,0.),stamp_ns=700)
        r.append_bound_image(frame())
        r.admit_decision(decision('active'),now_ns=800)
        now=800+int(seconds*1e9)
        r.update_apply_pose(Pose2D(1.,2.,0.),stamp_ns=now-1)
        result='';command=None
        try:
            out=r.step(frame(stamp_ns=now-1),now_ns=now)
            command=out.command.action if out.command else None
        except PixNavRuntimeError as error:result=str(error)
        rows.append(dict(seconds_since_admission=seconds,runtime_ttl_ms=r.ttl_ms,
                         max_policy_steps=r._active_max_go_steps,command=command,
                         error=result,worker_step_calls=len(worker.step_calls)))
    finally:node.destroy_node();rclpy.shutdown()
assert rows[0]['command']=='move_forward'
assert all('LOCAL_POLICY_EXPIRED' in r['error'] and r['worker_step_calls']==0 for r in rows[1:])
Path('/out/direct_lifetime.json').write_text(json.dumps(dict(cases=rows,physical_commands=0,
 interpretation='300s active local session lifetime persists despite 1800s operator episode cap. These are different layers. Existing <71s Direct model STOP trials were not caused by this expiry. No runtime settings were changed.'),indent=2)+'\n')
print(json.dumps(rows,indent=2))
