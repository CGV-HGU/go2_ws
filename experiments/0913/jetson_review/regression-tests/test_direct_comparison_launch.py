"""Evaluate launch choices without starting a node or actuator transport."""
import importlib.util
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


@pytest.mark.parametrize('mode', ['full', 'direct_goal'])
def test_direct_final_goal_does_not_launch_vlm_sweep_or_inherit_waypoint_cap(mode):
    path=Path(os.environ.get('S2E_TEST_ROBOT_ESCAPE_LAUNCH', str(Path(__file__).resolve().parents[1]/'launch/robot_escape.launch.py')))
    spec=importlib.util.spec_from_file_location('comparison_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    description=module.generate_launch_description()
    context=LaunchContext()
    context.launch_configurations.update(navigation_mode=mode,sensor_input_enabled='true',
        direct_goal_max_policy_steps='500',look_execution='forward_0p1',
        rotate_min_timeout_s='15',vlm_backend='live_vlm')
    names=[]
    for action in description.entities:
        if action.condition is not None and not action.condition.evaluate(context):continue
        if isinstance(action,Node):
            executable=action.node_executable
            names.append(executable if isinstance(executable,str) else perform_substitutions(context,executable))
        elif isinstance(action,SetEnvironmentVariable):action.execute(context)
    assert 'pixnav_runtime_node' in names and 'go2_action_executor' in names
    if mode=='direct_goal':
        assert 'direct_goal_pixnav' in names
        assert 'vlm_node' not in names and 'sweep_scheduler_node' not in names
        assert context.environment['PIXNAV_MAX_GO_STEPS']=='500'
    else:
        assert 'vlm_node' in names and 'sweep_scheduler_node' in names
        assert 'direct_goal_pixnav' not in names
        assert context.environment['PIXNAV_MAX_GO_STEPS']=='50'
    assert context.environment['PIXNAV_LOOK_EXECUTION']=='forward_0p1'
