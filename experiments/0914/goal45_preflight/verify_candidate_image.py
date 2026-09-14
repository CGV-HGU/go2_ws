"""Check installed candidate source and parameter wiring without actuator I/O."""
import hashlib
import inspect
import json
import math
from pathlib import Path
from unittest.mock import Mock

import rclpy
from s2e_vlm_robot import action_executor_node, motion

action_executor_node.SportClient = lambda *args: Mock()
rclpy.init(args=['--ros-args', '-p', 'rotate_settle_spread_deg:=1.0'])
node = action_executor_node.ActionExecutorNode()
assert not node._motion_enabled
assert node._limits.rotation_settle_spread == math.radians(1.)
assert node._limits.stop_yaw_rate == .1
assert node._limits.angle_tolerance == math.radians(3.)
hashes = {}
for module in (motion, action_executor_node):
    p = Path(inspect.getfile(module))
    expected = Path('/out/candidate') / p.name
    assert p.read_bytes() == expected.read_bytes()
    hashes[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
launch = Path('/opt/s2e-robot-minimal/share/s2e_vlm_robot/launch/robot_escape.launch.py')
assert launch.read_bytes() == Path('/out/candidate/robot_escape.launch.py').read_bytes()
assert 'DeclareLaunchArgument("rotate_settle_spread_deg"' in launch.read_text()
assert '"rotate_settle_spread_deg": ParameterValue(LaunchConfiguration("rotate_settle_spread_deg"), value_type=float)' in launch.read_text()
hashes[launch.name] = hashlib.sha256(launch.read_bytes()).hexdigest()
node.destroy_node()
rclpy.shutdown()
Path('/out/candidate_image_validation.json').write_text(json.dumps({
    'passed': True, 'source_sha256': hashes, 'configured_spread_deg': 1.,
    'stop_yaw_rate_unchanged': .1, 'angle_tolerance_unchanged_deg': 3.,
    'motion_enabled': False, 'physical_commands_sent': 0,
    'scope': 'Installed immutable candidate image, actual node parameter constructor, mocked Sport, launch parameter source wiring. Not launched navigation or live sensor test.'
}, indent=2)+'\n')
