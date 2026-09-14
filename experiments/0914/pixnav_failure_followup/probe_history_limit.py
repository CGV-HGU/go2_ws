"""Exercise deployed PixelNav history policy with a constant fake network.

Not a checkpoint-performance test. No robot, ROS, network, or model weight changes.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types

import numpy as np
import torch
from s2e_vlm_core.pixnav_native import PixelNavAgent
import s2e_vlm_core.pixnav_native as native

assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
torch.set_num_threads(1)


class ConstantNetwork:
    def eval(self):
        return self

    def __call__(self, goal_mask, goal_image, history):
        n = history.shape[1]
        action = torch.zeros((1, n, 6), dtype=torch.float32)
        action[:, :, 1] = 10.
        return action, torch.ones((1, n, 1)), torch.full((1, n, 2), .5)


image = np.zeros((224, 224, 3), dtype=np.uint8)
mask = np.zeros((224, 224), dtype=np.uint8); mask[110:115, 110:115] = 1
rows = []
for early_stop in (True, False):
    agent = PixelNavAgent(network=ConstantNetwork(), expected_sha256=None,
                         policy_camera_mode='raw', fixed_camera_view_zoom=1., fixed_camera_view_on_look=False)
    agent.reset(image, mask)
    actions = []; error = None
    for step in range(65):
        try:
            result = agent.step(image.copy(), early_stop=early_stop)
            action = int(result[0]); actions.append(action)
            if action == 0:
                break
        except RuntimeError as exc:
            error = str(exc); break
    rows.append(dict(implementation='deployed_native', early_stop=early_stop,
                     network_raw_action_always=1, returned_actions=actions,
                     first_stop_call_1based=(actions.index(0)+1 if 0 in actions else None),
                     forward_actions_before_stop=sum(a == 1 for a in actions),
                     stop_reason=agent.last_step_reason, error=error))

sys.path.insert(0, '/references/official')
sys.modules['quaternion'] = types.ModuleType('quaternion')  # Imported but unused by step.
spec = importlib.util.spec_from_file_location('official_policy_agent', '/references/official/policy_agent.py')
official = importlib.util.module_from_spec(spec); spec.loader.exec_module(official)
agent = official.Policy_Agent.__new__(official.Policy_Agent)
agent.image_size = 224; agent.max_token_length = 64; agent.device = 'cpu'
agent.network = ConstantNetwork(); agent.reset(image, mask)
actions = []
for i in range(33):
    actions.append(int(agent.step(image.copy(), early_stop=True)[0]))
rows.append(dict(implementation='official_reference', early_stop=True,
                 network_raw_action_always=1, returned_actions=actions,
                 first_stop_call_1based=actions.index(0)+1,
                 forward_actions_before_stop=sum(a == 1 for a in actions)))
assert rows[0]['first_stop_call_1based'] == rows[2]['first_stop_call_1based'] == 33
assert rows[0]['forward_actions_before_stop'] == rows[2]['forward_actions_before_stop'] == 32
assert rows[0]['stop_reason'] == 'POLICY_HISTORY_LIMIT'
assert rows[1]['forward_actions_before_stop'] == 64 and 'history exhausted' in rows[1]['error']
out = dict(scope='Session/history wrapper behavior with fixed forward logits; NOT model inference or real driving.',
           raw_network_action=1, production_source=str(native.__file__),
           production_source_sha256=hashlib.sha256(Path(native.__file__).read_bytes()).hexdigest(),
           official_source_sha256=hashlib.sha256(Path('/references/official/policy_agent.py').read_bytes()).hexdigest(),
           physical_commands_sent=0, checkpoint_loaded=False, cases=rows,
           implication='Default early_stop permits at most 32 executed actions in a single session, despite the outer 500-step setting. Nominal all-forward travel is 32*0.25=8m; turns/look substitutions reduce it. Disabling early_stop alone then meets the 64-observation hard limit.')
Path('/out/history_limit.json').write_text(json.dumps(out, indent=2)+'\n')
print(json.dumps({r['implementation']+'_'+str(r['early_stop']):
                  {k: v for k, v in r.items() if k not in ('returned_actions', 'implementation')}
                  for r in rows}, indent=2))
