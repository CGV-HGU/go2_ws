#!/usr/bin/env python3
"""Read-only current launch/goal contract audit; no physical readiness claim."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from datetime import datetime, timezone

import yaml

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main(runtime, output):
    campaign = runtime / '.local-data/recording-five-goals-recaptured5-20260914'
    source = runtime / 'scripts/fixed_start_goals.py'
    spec = importlib.util.spec_from_file_location('fixed_start_goals_audit', source)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    plan_file = campaign / 'goal-plan.json'
    plan = json.loads(plan_file.read_text())
    module.validate_plan(plan)
    goals = []; transforms = []
    for label in ('1', '2', '3'):
        g = module.validate_plan(plan, label); xy = g['point_goal_xy_m']
        goals.append(dict(label=label, x_m=xy['x'], y_m=xy['y'],
                          initial_euclidean_distance_m=math.hypot(xy['x'], xy['y']),
                          initial_bearing_deg=math.degrees(math.atan2(xy['y'], xy['x'])),
                          final_yaw_scored=False, physical_visibility_verified=False))
        for origin in ((0., 0., 0.), (3.2, -1.7, math.pi/2), (-8., 2., -math.pi/2), (1., 2., math.pi)):
            out = module.goal_in_odometry(plan, label, origin)
            dx, dy = out[0]-origin[0], out[1]-origin[1]
            recovered = [math.cos(origin[2])*dx+math.sin(origin[2])*dy,
                         -math.sin(origin[2])*dx+math.cos(origin[2])*dy]
            error = math.hypot(recovered[0]-xy['x'], recovered[1]-xy['y'])
            assert error < 1e-12
            transforms.append(dict(label=label, episode_origin=list(origin), roundtrip_error_m=error))
    snapshots = []; hashes = {str(source): sha(source), str(plan_file): sha(plan_file)}
    for label in ('1', '2', '3'):
        for mode in ('full', 'direct_goal'):
            p = campaign / 'episodes' / (mode+'-goal-'+label+'-002')
            mf = p/'manifest.json'; cf = p/'compose.trial.yaml'; pf = p/'goal-plan.json'
            m = json.loads(mf.read_text()); compose = yaml.safe_load(cf.read_text())
            conditions = m['runtime_conditions']; node = compose['services']['escape']
            env = node.get('environment', {}); cmd = node['command'][-1]
            assert conditions['goal_label'] == label and conditions['navigation'] == mode
            assert conditions['success_radius_m'] == 1 and conditions['look_execution'] == 'forward_0p2'
            assert conditions['forward_speed_mps'] == .5 and conditions['turn_speed_radps'] == .8
            assert conditions['localization_method'] == 'fixed_start_odometry'
            assert conditions['icp_corrections_used'] is False
            assert 'navigation_mode:='+mode in cmd and 'pose_source:=odometry' in cmd
            assert node['image'] == m['images']['escape']
            assert sha(pf) == sha(plan_file)
            started = (p/'START_GOAL').exists() or (p/'full5m/runtime/result.json').exists()
            assert not started, 'Prepared snapshot now has execution evidence; reselect inputs'
            for f in (mf, cf, pf): hashes[str(f)] = sha(f)
            snapshots.append(dict(run=p.name, navigation=mode, goal_label=label,
                goal_xy_m=module.validate_plan(plan, label)['point_goal_xy_m'],
                navigation_image=m['images']['escape'], look_execution=conditions['look_execution'],
                forward_speed_mps=conditions['forward_speed_mps'], turn_speed_radps=conditions['turn_speed_radps'],
                success_radius_m=conditions['success_radius_m'], max_duration_s=conditions['max_duration_s'],
                final_yaw_requested=m['final_yaw_requested'], icp_corrections_used=False,
                direct_max_policy_steps=conditions.get('direct_goal_max_policy_steps'),
                local_policy_ttl_ms_explicit_override=env.get('PIXNAV_LOCAL_POLICY_TTL_MS'),
                actual_drive_evidence=False))
    prior = json.loads((HERE.parent/'drive_timing_review/source_provenance.json').read_text())
    unchanged = {}
    for name in ('motion.py', 'action_executor_node.py', 'sport_client.py'):
        p = Path(prior[name]['source']); value = sha(p)
        unchanged[name] = value == prior[name]['sha256']; hashes[str(p)] = value
    assert all(unchanged.values())
    pix = runtime/'src/s2e_vlm_nodes/s2e_vlm_nodes/runtime/pixnav.py'
    pix_text = pix.read_text(); hashes[str(pix)] = sha(pix)
    assert 'ttl_ms=int(_env_float("PIXNAV_LOCAL_POLICY_TTL_MS", 300_000.0))' in pix_text
    assert 'raise PixNavRuntimeError("LOCAL_POLICY_EXPIRED")' in pix_text
    for name in ('direct_goal.py', 'direct_goal_node.py'):
        p = runtime/'src/s2e_vlm_robot/s2e_vlm_robot'/name; hashes[str(p)] = sha(p)
    actual_direct = []
    for c in sorted((runtime/'.local-data').glob('recording-five-goals*')):
        for p in sorted((c/'episodes').glob('direct_goal-goal-*')):
            f = p/'full5m/runtime/result.json'
            if not f.exists(): continue
            m = json.loads(f.read_text()); hashes[str(f)] = sha(f)
            actual_direct.append(dict(campaign=c.name, run=p.name, goal_label=m.get('goal_label'),
                                      goal_published=m.get('goal_published', False), reason=m.get('reason')))
    launched = [x for x in actual_direct if x['goal_published']]
    assert len(launched) == 5 and {str(x['goal_label']) for x in launched} == {'4'}
    result = dict(checked_at_utc=datetime.now(timezone.utc).isoformat(),
        scope='Current goal 1..3 prepared profiles and pure fixed-start coordinate conversion; not closed-loop navigation.',
        current_prepared_profiles=6, coordinate_roundtrip_cases=12, contract_checks_passed=True,
        selected_goal_geometry=goals, roundtrip_results=transforms, prepared=snapshots,
        actual_direct_inventory=actual_direct, known_fault_source_files_unchanged=unchanged,
        direct_policy_ttl_default_ms=300000,
        known_unfixed_issues=['rotation_settle_timeout', 'blocking_transport_sensor_processing_contention',
                             'direct_policy_300s_vs_episode_1800s'],
        possible_for_goals_1_2_3=True, physical_visibility_verified=False,
        code_proven_correct_for_all_driving=False, runtime_modified=False,
        note='Same shared executor/runtime paths serve all goal labels. The three reproduced faults were not fixed by this quantitative evaluation.',
        source_sha256=hashes)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(profiles=6, transforms=12, direct_started=len(launched),
                         unresolved_fault_sources_unchanged=all(unchanged.values()))))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root', type=Path, default=Path('/home/unitree/s2e-vlm-async-framework-minimal'))
    p.add_argument('--output', type=Path, default=HERE/'current_logic_audit.json')
    a = p.parse_args(); main(a.runtime_root, a.output)
