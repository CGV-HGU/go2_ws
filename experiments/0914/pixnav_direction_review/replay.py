"""Replay recorded Direct PixelNav inputs against two unchanged reference agents.

Runs without ROS or network. Applies the deployed calibrated camera transform
to both references before their own preprocessing. This checks inference parity
under the robot input contract, not the correctness of that contract or driving.
"""
import ast
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import types

import cv2
import numpy as np
import torch
from s2e_vlm_core.pixnav_adapter import array_evidence_ref
from s2e_vlm_core.pixnav_native import PixelNavAgent, EXPECTED_CHECKPOINT_SHA256

OUT = Path('/out')
REFERENCES = Path('/references')
EPISODES = Path('/episodes')
CHECKPOINT = Path('/models/pixelnav_A.ckpt')


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def unused_quaternion(name):
    if name.startswith('__'):
        raise AttributeError(name)
    raise RuntimeError('Reference unexpectedly used quaternion: ' + name)


def reference_agent(kind):
    source = REFERENCES / kind
    for filename in ('policy_agent.py', 'policy_network.py'):
        tree = ast.parse((source / filename).read_text())
        assert not any(isinstance(n, ast.Name) and n.id == 'quaternion' for n in ast.walk(tree))
    shim = types.ModuleType('quaternion')
    shim.__getattr__ = unused_quaternion
    sys.modules['quaternion'] = shim
    settings = types.ModuleType('settings')
    settings.DEFAULT_DEVICE = 'cuda'
    settings.POLICY_CHECKPOINT = str(CHECKPOINT)
    sys.modules['settings'] = settings
    sys.modules['policy_network'] = load(source / 'policy_network.py', kind + '_network')
    module = load(source / 'policy_agent.py', kind + '_agent')
    cls = getattr(module, 'PolicyAgent' if kind == 'upstream' else 'Policy_Agent')
    return cls(model_path=str(CHECKPOINT), max_token_length=64, image_size=224, device='cuda')


def capture(agent):
    def hook(module, args, outputs):
        agent.audit_outputs = [x[0, agent.predict_length].detach().cpu().numpy().copy() for x in outputs]
    return agent.network.register_forward_hook(hook)


def main():
    started = time.monotonic()
    episodes = sorted(EPISODES.glob('direct_goal-goal-4-00[2-6]'))
    assert len(episodes) == 5
    assert sha(CHECKPOINT) == EXPECTED_CHECKPOINT_SHA256
    contracts = [json.loads((p / 'camera-contract.json').read_text()) for p in episodes]
    assert all(c == contracts[0] for c in contracts)
    report = {'scope': __doc__, 'checkpoint_sha256': EXPECTED_CHECKPOINT_SHA256,
              'torch_version': torch.__version__, 'opencv_version': cv2.__version__,
              'robot_commands_published': 0, 'closed_loop_validation': False,
              'references': {}, 'input_files': [], 'source_sha256': {},
              'strict_output_tolerance': {'atol': 1e-5, 'rtol': 1e-5}}
    for kind in ('upstream', 'official'):
        for filename in ('policy_agent.py', 'policy_network.py'):
            report['source_sha256'][kind + '/' + filename] = sha(REFERENCES / kind / filename)
    native = PixelNavAgent(model_path=CHECKPOINT, device='cuda', camera_contract=contracts[0],
                          policy_camera_mode='calibrated_4_3', fixed_camera_view_zoom=1.,
                          fixed_camera_view_on_look=False,
                          allowed_actions='stop,move_forward,turn_left,turn_right,look_up,look_down')
    module_path = Path(sys.modules[PixelNavAgent.__module__].__file__)
    report['native_module_path'] = str(module_path)
    report['native_source_sha256'] = sha(module_path)
    native_hook = capture(native)
    for kind in ('upstream', 'official'):
        original = reference_agent(kind)
        assert set(original.network.state_dict()) == set(native.network.state_dict())
        assert all(torch.equal(v.cpu(), native.network.state_dict()[k].cpu())
                   for k, v in original.network.state_dict().items())
        hook = capture(original)
        results = []
        for episode in episodes:
            resets = list((episode / 'full5m/policy-inputs').glob('*/session_0001_reset.npz'))
            assert len(resets) == 1
            reset = resets[0]
            with np.load(reset) as data:
                goal, mask = data['goal_image'], data['goal_mask']
                saved_goal, saved_mask = data['policy_goal_image'], data['policy_goal_mask']
            native.reset(goal, mask)
            view = native._policy_camera_view
            original.reset(view.render(goal), view.goal_mask(mask))
            for agent in (native, original):
                np.testing.assert_array_equal(agent.goal_image, saved_goal)
                np.testing.assert_array_equal(agent.goal_mask, saved_mask)
            result = {'run': episode.name, 'reset_tensors_equal': True, 'steps': []}
            results.append(result)
            if kind == 'upstream':
                report['input_files'].append({'path': str(reset.relative_to(EPISODES)), 'sha256': sha(reset)})
            for meta_path in sorted(reset.parent.glob('session_0001_step_*.json')):
                meta = json.loads(meta_path.read_text())
                step = meta['step_index']
                with np.load(meta_path.with_suffix('.npz')) as data:
                    obs = data['observation']
                assert array_evidence_ref(obs) == meta['observation_ref']
                assert native.predict_length == original.predict_length == step
                assert meta['raw_logits'] == meta['effective_logits']
                assert not meta['collide']
                with torch.inference_mode():
                    na, _ = native.step(obs, collide=False, early_stop=True)
                    oa, _ = original.step(view.render(obs), collide=False, early_stop=True)
                np.testing.assert_array_equal(native.current_obs, original.current_obs)
                np.testing.assert_array_equal(native.input_image, original.input_image[:, :step+1])
                if kind == 'official':
                    assert np.count_nonzero(original.input_image[:, step+1:]) == 0
                a, b = native.audit_outputs, original.audit_outputs
                point = view.source_point((float(a[2][1])*view.width, float(a[2][0])*view.height))
                physical_goal = np.array([point[1]/obs.shape[0], point[0]/obs.shape[1]])
                actual = [a[0], a[1], physical_goal]
                expected = [meta['raw_logits'], meta['distance_prediction'], meta['goal_prediction']]
                row = {'step_index': step, 'recorded_action': meta['action'],
                       'native_action': int(na), 'reference_action': int(oa),
                       'input_tensors_equal': True,
                       'native_matches_recording': all(np.allclose(x,y,rtol=1e-5,atol=1e-5) for x,y in zip(actual, expected)),
                       'actions_equal': int(na) == int(oa) == meta['action'],
                       'outputs_allclose': all(np.allclose(x,y,rtol=1e-5,atol=1e-5) for x,y in zip(a,b)),
                       'max_abs_difference': {k: float(np.max(np.abs(x-y))) for k,x,y in zip(('logits','distance','goal'), a,b)},
                       'reference_raw_logits': b[0].tolist(), 'recorded_stop_source': meta['stop_source']}
                result['steps'].append(row)
                if kind == 'upstream':
                    for path in (meta_path, meta_path.with_suffix('.npz')):
                        report['input_files'].append({'path': str(path.relative_to(EPISODES)), 'sha256': sha(path)})
                if not row['actions_equal']:
                    result['stopped_at_first_action_divergence'] = True
                    break
            print(json.dumps({'reference': kind, 'run': episode.name, 'steps': len(result['steps']),
                              'action_mismatches': sum(not s['actions_equal'] for s in result['steps'])}), flush=True)
        flat = [s for r in results for s in r['steps']]
        report['references'][kind] = {'state_tensors_equal': True, 'runs': results,
            'summary': {'steps': len(flat), 'action_mismatches': sum(not s['actions_equal'] for s in flat),
                        'native_recording_mismatches': sum(not s['native_matches_recording'] for s in flat),
                        'strict_output_mismatches': sum(not s['outputs_allclose'] for s in flat),
                        'max_abs_difference': {k: max(s['max_abs_difference'][k] for s in flat) for k in ('logits','distance','goal')}}}
        (OUT / 'replay_results.json').write_text(json.dumps(report, indent=2)+'\n')
        hook.remove()
        del original
        torch.cuda.empty_cache()
    native_hook.remove()
    report['elapsed_s'] = time.monotonic() - started
    report['all_actions_match'] = all(r['summary']['steps'] == 118 and r['summary']['action_mismatches'] == 0
                                      for r in report['references'].values())
    report['strict_numeric_parity'] = all(r['summary']['strict_output_mismatches'] == 0 for r in report['references'].values())
    (OUT / 'replay_results.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:r['summary'] for k,r in report['references'].items()}, indent=2), flush=True)
    return 0 if report['all_actions_match'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
