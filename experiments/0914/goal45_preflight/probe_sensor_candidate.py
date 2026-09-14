"""Actual candidate ROS callbacks, synthetic sensors/recording transport only."""
from pathlib import Path
import json
import types

source = Path('/previous/probe_callback_delay.py').read_text()
source = source[:source.index('\nresults=[]')]
module = types.ModuleType('candidate_sensor_probe_base')
exec(compile(source, '<prior-probe-functions>', 'exec'), module.__dict__)
results = []
for delay in (.08, .095, .095, .15):
    result = module.run(delay, separate=False, command_rate=20.)
    results.append(result)
    print(json.dumps({k:v for k,v in result.items() if k != 'samples'}), flush=True)
Path('/out/sensor_candidate.json').write_text(json.dumps({
    'cases': results, 'physical_commands_sent': 0,
    'scope': 'Candidate actual ROS callbacks and existing admission path; synthetic 30Hz odom/pose/RGB, 3 threads, fake active macro and fake Sport. 150ms case tests scheduling only beyond real 100ms transport timeout.',
    'all_no_stale': all(not r['fault'] for r in results),
}, indent=2)+'\n')
assert all(not r['fault'] for r in results)
