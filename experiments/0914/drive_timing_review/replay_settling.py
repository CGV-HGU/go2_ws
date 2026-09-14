"""Exercise installed pure motion logic on recorded yaw, without ROS or actuation.

Linear speed is optimistically zero because controller-filtered linear speed was
not logged. Timeout even under this assumption proves angular acceptance blocks.
Alternate acceptance is NOT proof of physical stop or subsequent driving success.
"""
from dataclasses import replace
import hashlib
import inspect
import json
import math
from pathlib import Path
import types

import s2e_vlm_robot.motion as installed

OUT=Path('/out')
SOURCE=Path('/evidence/recaptured5/full-goal-5-001/motion_events.json')
all_events=json.loads(SOURCE.read_text())
terminal=all_events[-1]
start=terminal['log_unix_s']-terminal['elapsed_s']
ticks=[e for e in all_events if e['event']=='rotate_command' and e['log_unix_s']>start]
source=inspect.getsource(installed)


def replay(module, rate=.1, speed=.0, tail=0):
    limits=module.MotionLimits(angular_speed=.8,minimum_turn_speed=.8,
      rotation_settle_window=.35,rotation_recoil_limit=math.radians(10),stop_yaw_rate=rate)
    first=module.MotionSample(ticks[0]['odom_stamp_ns']-1,0,0,0,0,speed,0)
    m=module.MeasuredMotion(kind='rotate',target=terminal['target'],sample=first,now=0,
                           timeout=ticks[0]['tracking_budget_s'],limits=limits)
    output=None;first_divergence=None;observed=None
    for e in ticks:
        t=e['elapsed_s']
        sample=module.MotionSample(e['odom_stamp_ns'],t,0,0,e['measured_progress'],speed,e['measured_yaw_rate_radps'])
        output=m.step(sample,now=t)
        if abs(output.angular-e['angular_command_radps'])>1e-7 and first_divergence is None:first_divergence=t
        if output.terminal:
            observed=t;break
    if not output.terminal:
        t=terminal['elapsed_s'];sample=module.MotionSample(terminal['odom_stamp_ns'],t,0,0,terminal['measured_progress'],speed,0)
        output=m.step(sample,now=t);observed=t
    remaining=[e for e in ticks if e['elapsed_s']>=observed]
    return dict(terminal=output.terminal,elapsed_s=observed,assumed_linear_speed=speed,
                first_angular_command_difference_s=first_divergence,
                remaining_error_deg=math.degrees(m.target-output.progress),
                max_later_recorded_error_deg=max((abs(math.degrees(e['target']-e['measured_progress'])) for e in remaining),default=None))


results=[]
for spread in [.25,.5,1.0]:
    module=installed
    if spread!=.25:
        # A temporary in-memory copy changes only this diagnostic threshold.
        assert source.count('math.radians(.25)')==1
        module=types.ModuleType('diagnostic_motion')
        import sys
        sys.modules[module.__name__]=module
        exec(compile(source.replace('math.radians(.25)',f'math.radians({spread})'),'<offline-only>', 'exec'),module.__dict__)
    for rate in [.1,.15]:
        r=replay(module,rate)
        r.update(spread_deg=spread,yaw_rate_limit=rate)
        results.append(r)
assert results[0]['terminal']=='MOTION_TIMEOUT'
assert results[0]['first_angular_command_difference_s'] is None
assert all(r['terminal']=='MOTION_TIMEOUT' for r in results if r['spread_deg']==.25)
# Keeping the unrecorded physical speed above the stop criterion must prevent ACK.
moving=replay(module,.15,.08)
assert moving['terminal']=='MOTION_TIMEOUT'

# The installed implementation already combines forward velocity and heading hold.
limits=installed.MotionLimits(angular_speed=.8,minimum_turn_speed=.8,rotation_settle_window=.35)
initial=installed.MotionSample(1,0,0,0,0,0,0)
m=installed.MeasuredMotion(kind='translate',target=.25,sample=initial,now=0,timeout=6,limits=limits)
out=m.step(installed.MotionSample(100000001,.1,.001,0,math.radians(5),.1,0),now=.1)
assert out.linear>0 and out.angular<0

result=dict(recorded_tick_count=len(ticks),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
 installed_motion_path=inspect.getfile(installed),installed_motion_sha256=hashlib.sha256(source.encode()).hexdigest(),
 installed_baseline_reproduced=True, results=results,nonstationary_speed_still_blocks=moving,
 concurrent_heading_hold_test=dict(linear=out.linear,angular=out.angular),
 physical_commands=0,production_source_modified=False,
 limitation='Exact angular command replay with synthetic zero translation/speed, not exact full odometry/controller replay. Earlier ACK candidates require measured speed validation and physical testing; no threshold recommendation or runtime change is implied.')
(OUT/'settling_replay.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
