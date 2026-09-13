#!/usr/bin/env python3
"""Capture goals relative to a marked physical start. Never publish or move."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import uuid


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def odom_pose(snapshot, now, boot_id):
    if snapshot.get('host_boot_id') != boot_id:
        raise ValueError('Odometry snapshot belongs to another boot')
    if not 0 <= now-snapshot['checked_unix'] <= 1.:
        raise ValueError('Odometry recorder is stale')
    owners = snapshot.get('odom_publisher_gids', [])
    if len(owners) != 1 or not owners[0]:
        raise ValueError('Require one odometry publisher')
    value = snapshot['topics']['odom']
    if value['frame'] != 'odom' or not -.1 <= now-value['stamp_unix'] <= .5 or not 0 <= now-value['received_unix'] <= .5:
        raise ValueError('Require fresh odometry in odom frame')
    p = value['pose']['pose']; q = p['orientation']; xyz = p['position']
    if not all(finite(v) for v in [xyz[k] for k in ('x', 'y', 'z')]+[q[k] for k in ('x', 'y', 'z', 'w')]):
        raise ValueError('Invalid odometry pose')
    if abs(sum(q[k]**2 for k in ('x', 'y', 'z', 'w'))-1) > .01:
        raise ValueError('Invalid odometry quaternion')
    yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
    return [xyz['x'], xyz['y'], yaw]


def make_origin(snapshot, now, boot_id):
    return dict(schema_version=1, origin_id=uuid.uuid4().hex,
                frame_id='fixed_start', pose_source='fixed_start_odometry',
                pose_in_odom=odom_pose(snapshot, now, boot_id),
                host_boot_id=boot_id, odom_publisher_gid=snapshot['odom_publisher_gids'][0],
                captured_unix=now, pose_stamp_unix=snapshot['topics']['odom']['stamp_unix'],
                physical_start_confirmed_by_operator=True,
                absolute_map_localization_verified=False)


def capture_goal(origin, label, snapshot, now, boot_id):
    if str(label) not in ('1', '2', '3', '4', '5'):
        raise ValueError('Goal label must be 1 through 5')
    current = odom_pose(snapshot, now, boot_id)
    if origin['host_boot_id'] != boot_id or origin['odom_publisher_gid'] != snapshot['odom_publisher_gids'][0]:
        raise ValueError('Odometry epoch changed during goal collection; re-establish the marked start')
    if snapshot['topics']['odom']['stamp_unix'] < origin['pose_stamp_unix']:
        raise ValueError('Odometry timestamp reset during goal collection')
    x, y, yaw = origin['pose_in_odom']; dx, dy = current[0]-x, current[1]-y
    return dict(label=str(label), frame_id='fixed_start',
                point_goal_xy_m={'x': math.cos(yaw)*dx+math.sin(yaw)*dy,
                                 'y': -math.sin(yaw)*dx+math.cos(yaw)*dy},
                yaw_rad=math.atan2(math.sin(current[2]-yaw), math.cos(current[2]-yaw)),
                origin_sha256=fingerprint(origin), captured_unix=now,
                pose_stamp_unix=snapshot['topics']['odom']['stamp_unix'],
                pose_in_odom=current, pose_source='fixed_start_odometry',
                absolute_map_localization_verified=False,
                physical_goal_accuracy_verified=False)


def make_plan(origin, goals):
    plan = dict(schema_version=1, pose_source='fixed_start_odometry',
                frame_id='fixed_start', origin=origin, origin_sha256=fingerprint(origin),
                goals=sorted(goals, key=lambda g: int(g['label'])),
                requires_marked_start_before_each_episode=True,
                reanchor_at_dispatch=True, icp_corrections_used=False,
                success_radius_m=1., final_yaw_requested=False,
                absolute_map_localization_verified=False)
    validate_plan(plan)
    return plan


def validate_plan(plan, label=None):
    if (plan.get('schema_version') != 1 or plan.get('pose_source') != 'fixed_start_odometry'
            or plan.get('frame_id') != 'fixed_start' or plan.get('icp_corrections_used') is not False
            or plan.get('requires_marked_start_before_each_episode') is not True
            or plan.get('reanchor_at_dispatch') is not True):
        raise ValueError('Invalid fixed-start coordinate contract')
    if plan.get('origin_sha256') != fingerprint(plan['origin']):
        raise ValueError('Fixed-start origin changed')
    origin = plan['origin']
    if (origin.get('frame_id') != 'fixed_start' or origin.get('pose_source') != 'fixed_start_odometry'
            or origin.get('physical_start_confirmed_by_operator') is not True
            or len(origin.get('pose_in_odom', [])) != 3
            or not all(finite(v) for v in origin['pose_in_odom'])):
        raise ValueError('Invalid captured fixed start')
    seen = set()
    for goal in plan['goals']:
        tag = goal.get('label')
        if tag not in ('1', '2', '3', '4', '5') or tag in seen:
            raise ValueError('Invalid or duplicate goal label')
        seen.add(tag)
        if (goal.get('frame_id') != 'fixed_start' or goal.get('pose_source') != 'fixed_start_odometry'
                or goal.get('origin_sha256') != plan['origin_sha256']):
            raise ValueError('Goal was captured against another origin')
        if not all(finite(goal.get('point_goal_xy_m', {}).get(k)) for k in ('x', 'y')):
            raise ValueError('Invalid fixed-start goal coordinates')
    if not seen or (label is not None and str(label) not in seen):
        raise ValueError('Requested saved goal is unavailable')
    if plan.get('success_radius_m') != 1. or plan.get('final_yaw_requested') is not False:
        raise ValueError('Unexpected arrival criterion')
    return next((g for g in plan['goals'] if g['label'] == str(label)), None)


def goal_in_odometry(plan, label, episode_start):
    """Bind once at the marked start, even after reboot; never reuse old odom XY."""
    goal = validate_plan(plan, label)
    if len(episode_start) != 3 or not all(finite(v) for v in episode_start):
        raise ValueError('Invalid episode start pose')
    x, y, yaw = episode_start; p = goal['point_goal_xy_m']
    return [x+math.cos(yaw)*p['x']-math.sin(yaw)*p['y'],
            y+math.sin(yaw)*p['x']+math.cos(yaw)*p['y']]


def validate_start_confirmation(plan, confirmation, now):
    validate_plan(plan)
    if (confirmation.get('fixed_start_confirmed') is not True
            or confirmation.get('origin_id') != plan['origin']['origin_id']
            or not finite(confirmation.get('confirmed_unix'))
            or not 0 <= now-confirmation['confirmed_unix'] <= 300):
        raise ValueError('Require recent operator confirmation at this goal set\'s marked start and heading')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('origin', 'capture', 'plan'))
    parser.add_argument('--goal-dir', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--label', choices=('1', '2', '3', '4', '5'))
    parser.add_argument('--start-confirmed', action='store_true')
    args = parser.parse_args(); root = args.goal_dir.resolve()
    if args.action == 'plan':
        origin = json.loads((root/'origin.json').read_text())
        goals = [json.loads(p.read_text()) for p in sorted(root.glob('[1-5].json'))]
        result = make_plan(origin, goals); target = root/'goal-plan.json'
        # Explicitly derived output; individual origin/goal captures stay immutable.
        target.write_text(json.dumps(result, indent=2)+'\n'); print(target); return
    if args.action == 'origin' and not args.start_confirmed:
        parser.error('Origin requires operator confirmation of the marked physical start and heading')
    if args.action == 'capture' and not args.label:
        parser.error('Capture requires --label')
    if args.run_dir is None:
        parser.error('Capture requires the live recorder --run-dir')
    requested = time.time(); deadline = time.monotonic()+3
    while True:
        snapshot = json.loads((args.run_dir/'latest.json').read_text())
        if snapshot['checked_unix'] >= requested and snapshot['topics']['odom']['stamp_unix'] >= requested-.1:
            break
        if time.monotonic() >= deadline:
            raise ValueError('No fresh odometry after capture request')
        time.sleep(.05)
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    result = (make_origin(snapshot, time.time(), boot) if args.action == 'origin' else
              capture_goal(json.loads((root/'origin.json').read_text()), args.label, snapshot, time.time(), boot))
    root.mkdir(parents=True, exist_ok=True); target = root/('origin.json' if args.action == 'origin' else args.label+'.json')
    evidence=target.with_name(target.stem+'.snapshot.json')
    if target.exists() or evidence.exists():
        raise FileExistsError('Origin/goal capture already exists; preserve it and use a new goal set')
    payload=json.dumps(snapshot,indent=2)+'\n'
    result.update(source_snapshot_file=evidence.name,source_snapshot_sha256=hashlib.sha256(payload.encode()).hexdigest())
    with evidence.open('x') as stream:stream.write(payload)
    with target.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(target); print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
