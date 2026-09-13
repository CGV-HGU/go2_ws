"""Exercise the installed log writer only; no ROS node or network request."""
import os,json,base64,time,hashlib
from pathlib import Path
from types import SimpleNamespace
from s2e_vlm_nodes.runtime.vlm import VlmMockNode
assert set(os.listdir('/sys/class/net'))=={'lo'}
out=Path('/work/.local-data/five-goal-recording-prep-20260913')
rgb=Path('/work/.local-data/fixed-start-goals-20260913/prepared/goal-1-002/full5m/observation-recording/rgb_000000.ppm').read_bytes()
request={'model':'offline-serialization-only','messages':[{'role':'user','content':[{'type':'text','text':'offline log preservation test'},{'type':'image_url','image_url':{'url':'data:image/x-portable-pixmap;base64,'+base64.b64encode(rgb).decode()}}]}]}
p=out/'model-input-log-private.jsonl'
node=SimpleNamespace(model_call_log_path=str(p),model_call_log_include_request_payload=True,dynamic_episode_index=0)
elapsed=[]
for i in range(5):
 t=time.monotonic();VlmMockNode._append_model_call(node,{'call_index':i,'episode_id':'offline-'+str(i),'request':request,'response':{'content':'recorded'},'latency_s':0.});elapsed.append(time.monotonic()-t)
rows=[json.loads(s) for s in p.read_text().splitlines()]
assert len(rows)==5 and len({r['episode_id'] for r in rows})==5
assert all(r['request']==request and r['input_image_count']==1 for r in rows)
result=dict(passed=True,rows=len(rows),payload_image_bytes=len(rgb),image_sha256=hashlib.sha256(rgb).hexdigest(),writer_duration_s=elapsed,network_requests=0,scope='Installed writer round trip using recorded image bytes; not a VLM inference or new driving result')
(out/'model-input-log-check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
