"""Read MCAP file storage only. No ROS node or publisher is created."""
import json
from pathlib import Path
import rosbag2_py
ROOT=Path('/work/.local-data')
output=[]
for trial,bag in [('goal-2-001','native-inputs-fixed-start'),('goal-1-001','native-inputs-goal-1-001'),('goal-2-002','native-inputs-goal-2-002')]:
 p=ROOT/'prepared-daylight-map-20260913'/bag
 ev=[json.loads(l) for l in (ROOT/'fixed-start-goals-20260913/prepared'/trial/'full5m/runtime/events.jsonl').open()]
 import datetime
 t0=int(datetime.datetime.fromisoformat(next(e['utc'] for e in ev if e['event']=='goal_published')).timestamp()*1e9)
 t1=int(datetime.datetime.fromisoformat(next(e['utc'] for e in ev if e['event']=='direct_stop')).timestamp()*1e9)
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(p),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 reader.set_filter(rosbag2_py.StorageFilter(topics=['/s2e/mapping/cloud','/s2e/robot/sensors/cloud']))
 n=0;within=0;first=None;last=None
 while reader.has_next():
  topic,data,t=reader.read_next();n+=1;first=t if first is None else first;last=t
  within+=int(t0<=t<=t1)
 output.append(dict(trial=trial,bag=str(p),total_cloud=n,cloud_during_drive=within,first_cloud_unix=first/1e9 if first else None,last_cloud_unix=last/1e9 if last else None,drive_start_unix=t0/1e9,drive_stop_unix=t1/1e9))
 print(json.dumps(output[-1]),flush=True)
(ROOT/'jetson-post-drive-20260913/cloud-time-coverage.json').write_text(json.dumps(output,indent=2)+'\n')
