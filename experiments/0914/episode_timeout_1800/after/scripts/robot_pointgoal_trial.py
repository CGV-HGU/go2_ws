#!/usr/bin/env python3
"""Supervised Full or diagnostic PointGoal trial. Running this script can command motion."""
import argparse,datetime,json,math,os,signal,time
parser=argparse.ArgumentParser()
target=parser.add_mutually_exclusive_group(required=True)
target.add_argument('--distance-m',type=float)
target.add_argument('--map-plan',type=str)
target.add_argument('--fixed-start-plan',type=str)
parser.add_argument('--goal-label',choices=('1','2','3','4','5'))
parser.add_argument('--localization-run',default='/localization')
parser.add_argument('--localization-state-root',default='/localization-state')
parser.add_argument('--max-duration-s',type=float,default=180.)
parser.add_argument('--navigation-mode',choices=('full','direct_goal'),default='full')
args=parser.parse_args()
planner_name='direct_goal_pixnav' if args.navigation_mode=='direct_goal' else 'vlm_node'
required_nodes=('go2_action_executor',planner_name,'pixnav_runtime_node','rtab_pointgoal_adapter')
if args.navigation_mode=='full':required_nodes+=('sweep_scheduler_node',)
if args.distance_m is not None and (not math.isfinite(args.distance_m) or args.distance_m<=0): raise ValueError('positive finite distance required')
if not math.isfinite(args.max_duration_s) or not 0<args.max_duration_s<=1800: raise ValueError('trial duration must be within (0,1800] seconds')
if (args.map_plan or args.fixed_start_plan) and not args.goal_label: parser.error('Saved goal plans require --goal-label')
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped
from capture_robot_goal import active_localization_manifest
from fixed_start_goals import validate_plan, goal_in_odometry, validate_start_confirmation
from std_srvs.srv import SetBool
from rcl_interfaces.srv import GetParameters
from s2e_vlm_msgs.msg import NodeStatus,PointGoal,NavigationTask,HabitatActionCommand,HabitatActionResult,StampedPose
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init(args=['--ros-args','--log-level','error']); n=Node('escape_pointgoal_trial_operator')
folder=Path('/trial/full5m/runtime'); folder.mkdir(parents=True,exist_ok=True)
log=(folder/'events.jsonl').open('x',buffering=1)
latest={}; status={}; status_at={}; pg=None; task=None; armed=False; published=False; shutdown_ok=True; reason=''; start=time.monotonic(); results=[]
def event(kind,data):
 log.write(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_s=time.monotonic()-start,event=kind,data=data))+'\n')
def receive(kind):
 def cb(m):
  global pg,task
  latest[kind]=(time.monotonic(),m)
  if kind=='status': status[m.node_name]=m;status_at[m.node_name]=time.monotonic()
  if kind=='pointgoal': pg=m
  if kind=='task': task=m
  if kind=='result': results.append(message_to_ordereddict(m))
  if kind!='odom' or not hasattr(cb,'last') or time.monotonic()-cb.last>.05:
   event(kind,message_to_ordereddict(m));cb.last=time.monotonic()
 return cb
n.create_subscription(Odometry,'/s2e/robot/sensors/odom',receive('odom'),qos_profile_sensor_data)
n.create_subscription(StampedPose,'/s2e/odometry/pose',receive('control_pose'),qos_profile_sensor_data)
n.create_subscription(PointGoal,'/s2e/sensors/pointgoal',receive('pointgoal'),qos_profile_sensor_data)
n.create_subscription(NavigationTask,'/s2e/task',receive('task'),10)
for topic in ['/s2e/controller/status','/s2e/robot/observation_status','/s2e/robot/sensor_status','/s2e/status/'+planner_name,'/s2e/status/pixnav_runtime_node','/s2e/status/sweep_scheduler_node']:
 n.create_subscription(NodeStatus,topic,receive('status'),10)
n.create_subscription(HabitatActionCommand,'/s2e/controller/habitat_action',receive('command'),10)
n.create_subscription(HabitatActionResult,'/s2e/sim/action_result',receive('result'),10)
if args.map_plan:
 n.create_subscription(PoseWithCovarianceStamped,'/rtabmap/localization_pose',receive('localization'),qos_profile_sensor_data)
frame='map' if args.map_plan else 'odom'
pose_key='localization' if args.map_plan else 'odom'
pub=n.create_publisher(PointStamped,'/s2e/goal/'+frame,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
client=n.create_client(SetBool,'/s2e/robot/set_motion_enabled')
def call(value):
 if not client.wait_for_service(timeout_sec=3): raise RuntimeError('MOTION_SERVICE_UNAVAILABLE')
 future=client.call_async(SetBool.Request(data=value)); rclpy.spin_until_future_complete(n,future,timeout_sec=5)
 if not future.done() or future.result() is None: raise RuntimeError('MOTION_SERVICE_TIMEOUT')
 response=future.result(); event('motion_enable' if value else 'motion_disable',dict(success=response.success,message=response.message))
 print(json.dumps(dict(service='enable' if value else 'disable',success=response.success,message=response.message)),flush=True)
 if not response.success: raise RuntimeError(response.message)
def check_map_binding():
 pointer=json.loads((Path(args.localization_state_root)/'current-localization.json').read_text())
 manifest=json.loads((Path(args.localization_run)/'manifest.json').read_text())
 active_localization_manifest(pointer,manifest,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
 if plan['source_database_sha256']!=manifest['source_database_sha256']:
  raise RuntimeError('GOAL_MAP_DATABASE_MISMATCH')
 return manifest

def check_fixed_start_confirmation():
 confirmation=json.loads(Path('/trial/START_GOAL').read_text())
 validate_start_confirmation(plan,confirmation,time.time())
 return confirmation

def check_live_localization_contract():
 for service,expected in (
  ('/rtabmap/rtabmap/get_parameters',{'pub_loc_pose_only_when_localizing':True,'Mem/IncrementalMemory':'false','RGBD/LinearUpdate':'0','RGBD/AngularUpdate':'0'}),
  ('/rtab_pointgoal_adapter/get_parameters',{'pose_source':'localization'})):
  c=n.create_client(GetParameters,service)
  if not c.wait_for_service(timeout_sec=3): raise RuntimeError('LOCALIZATION_PARAMETER_SERVICE_UNAVAILABLE')
  f=c.call_async(GetParameters.Request(names=list(expected)))
  rclpy.spin_until_future_complete(n,f,timeout_sec=3)
  if not f.done() or f.result() is None: raise RuntimeError('LOCALIZATION_PARAMETER_SERVICE_TIMEOUT')
  values={k:(v.bool_value if v.type==1 else v.string_value) for k,v in zip(expected,f.result().values)}
  if values!=expected: raise RuntimeError('LOCALIZATION_CONFIGURATION_MISMATCH')
 if n.count_publishers('/rtabmap/localization_pose')!=1 or n.count_publishers('/s2e/robot/sensors/odom')!=1:
  raise RuntimeError('LOCALIZATION_SENSOR_OWNER_MISMATCH')

def current_position(max_age):
 if pose_key not in latest: raise RuntimeError('CURRENT_GOAL_FRAME_POSE_UNAVAILABLE')
 received,msg=latest[pose_key]
 age=(n.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)*1e-9
 if msg.header.frame_id!=frame or time.monotonic()-received>max_age or not -.1<=age<=max_age:
  raise RuntimeError('FRESH_GOAL_FRAME_POSE_UNAVAILABLE')
 p=msg.pose.pose;q=p.orientation
 if not all(math.isfinite(v) for v in (p.position.x,p.position.y,p.position.z,q.x,q.y,q.z,q.w)):
  raise RuntimeError('INVALID_GOAL_FRAME_POSE')
 if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:
  raise RuntimeError('INVALID_GOAL_FRAME_ORIENTATION')
 if args.map_plan:
  cov=msg.pose.covariance
  if not all(math.isfinite(v) for v in cov) or min(cov[i] for i in (0,7,35))<0 or sum(cov[i] for i in (0,7,35))>1.:
   raise RuntimeError('INVALID_LOCALIZATION_COVARIANCE')
 return p,age

def control_pose_ready(max_age=.5):
 if 'control_pose' not in latest: return False
 received,m=latest['control_pose']
 age=(n.get_clock().now().nanoseconds-m.header.stamp.sec*10**9-m.header.stamp.nanosec)*1e-9
 # Match the executor during readiness; reserve a tighter transport margin
 # only for the final service request, not as a sustained startup condition.
 return (m.header.frame_id=='robot_origin' and m.status=='OK'
         and time.monotonic()-received<max_age and -.1<=age<max_age)

def interrupt(*args): raise KeyboardInterrupt
def check_stop():
 if (folder/'STOP').exists(): raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
signal.signal(signal.SIGINT,interrupt)
try:
 check_stop()
 if args.fixed_start_plan:
  plan=json.loads(Path(args.fixed_start_plan).read_text())
  selected=validate_plan(plan,args.goal_label)
  check_fixed_start_confirmation()
  xy=selected['point_goal_xy_m']
  if math.hypot(xy['x'],xy['y'])<=plan['success_radius_m']:
   raise RuntimeError('FIXED_GOAL_ALREADY_WITHIN_ARRIVAL_RADIUS')
 if args.map_plan:
  plan=json.loads(Path(args.map_plan).read_text())
  if plan.get('pose_source')!='localization': raise RuntimeError('GOAL_PLAN_NOT_LOCALIZATION')
  selected=[g for g in plan['goals'] if g['label']==args.goal_label]
  if len(selected)!=1: raise RuntimeError('GOAL_LABEL_UNAVAILABLE')
  selected=selected[0]
  if selected['message']['header']['frame_id']!='map' or selected['topic']!='/s2e/goal/map': raise RuntimeError('GOAL_PLAN_FRAME_MISMATCH')
  if not all(math.isfinite(selected['message']['point'][k]) for k in ('x','y')): raise RuntimeError('INVALID_GOAL_COORDINATES')
  check_map_binding()
 until=time.monotonic()+45
 ready_since=None
 while time.monotonic()<until:
  check_stop()
  rclpy.spin_once(n,timeout_sec=.05)
  c=status.get('go2_action_executor')
  ready = control_pose_ready() and pose_key in latest and time.monotonic()-latest[pose_key][0] < (1.5 if args.map_plan else .3) and pub.get_subscription_count()>0 and c and c.state=='INHIBITED' and c.active_mode=='IDLE' and all(name in status and status[name].is_healthy and time.monotonic()-status_at[name]<3. for name in required_nodes)
  if ready:
   if ready_since is None: ready_since=time.monotonic()
   if time.monotonic()-ready_since>=1.5: break
  else: ready_since=None
 else: raise RuntimeError('CURRENT_ODOM_OR_IDLE_CONTROLLER_UNAVAILABLE')
 check_stop()
 if args.map_plan:
  check_map_binding()
  check_live_localization_contract()
 if args.fixed_start_plan:
  check_fixed_start_confirmation()
  c=n.create_client(GetParameters,'/rtab_pointgoal_adapter/get_parameters')
  if not c.wait_for_service(timeout_sec=3): raise RuntimeError('ADAPTER_PARAMETER_SERVICE_UNAVAILABLE')
  f=c.call_async(GetParameters.Request(names=['pose_source']))
  rclpy.spin_until_future_complete(n,f,timeout_sec=3)
  if not f.done() or f.result() is None or f.result().values[0].string_value!='odometry':
   raise RuntimeError('FIXED_START_REQUIRES_ODOMETRY_CONTROL')
  if n.count_publishers('/s2e/robot/sensors/odom')!=1: raise RuntimeError('ODOMETRY_SENSOR_OWNER_MISMATCH')
 # Parameter service discovery can outlive the previous readiness sample.
 until=time.monotonic()+5
 while not control_pose_ready(.25):
  check_stop()
  if time.monotonic()>=until: raise RuntimeError('FRESH_CONTROL_POSE_UNAVAILABLE')
  rclpy.spin_once(n,timeout_sec=.02)
 current_position(1.5 if args.map_plan else .3)
 armed=True; call(True)
 check_stop()
 # Capture the freshest pose after enable; use absolute odom to avoid an old session origin.
 rclpy.spin_once(n,timeout_sec=.05)
 p,age=current_position(1.5 if args.map_plan else .3)
 q=p.orientation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 goal=PointStamped();goal.header.stamp=n.get_clock().now().to_msg();goal.header.frame_id=frame
 if args.map_plan:
  check_map_binding()
  goal.point.x=selected['message']['point']['x'];goal.point.y=selected['message']['point']['y'];goal.point.z=0.
 elif args.fixed_start_plan:
  confirmation=check_fixed_start_confirmation()
  goal.point.x,goal.point.y=goal_in_odometry(plan,args.goal_label,[p.position.x,p.position.y,yaw])
  goal.point.z=p.position.z
  event('fixed_start_binding',dict(origin_id=plan['origin']['origin_id'],origin_sha256=plan['origin_sha256'],
   episode_start_in_odom=[p.position.x,p.position.y,yaw],saved_goal=selected,
   host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
   operator_confirmation=confirmation,icp_corrections_used=False))
 else:
  goal.point.x=p.position.x+args.distance_m*math.cos(yaw);goal.point.y=p.position.y+args.distance_m*math.sin(yaw);goal.point.z=p.position.z
 check_stop()
 goal_stamp_ns=goal.header.stamp.sec*10**9+goal.header.stamp.nanosec
 task=None;pg=None
 pub.publish(goal);published=True;sent=time.monotonic()
 details=dict(origin_x=p.position.x,origin_y=p.position.y,heading_rad=yaw,goal_x=goal.point.x,goal_y=goal.point.y,distance_m=math.hypot(goal.point.x-p.position.x,goal.point.y-p.position.y),continuing_original_goal=False,origin_age_s=age,frame=frame,goal_label=args.goal_label,success_criterion='distance within configured PointGoal radius; final yaw is not controlled',navigation_mode=args.navigation_mode,look_execution=os.environ.get('PIXNAV_LOOK_EXECUTION','physical'))
 if args.fixed_start_plan:
  details.update(localization_method='fixed_start_odometry',fixed_start_origin_id=plan['origin']['origin_id'],fixed_start_goal_xy=selected['point_goal_xy_m'],physical_arrival_verified=False)
 (folder/'goal.json').write_text(json.dumps(details,indent=2));event('goal_published',details);print(json.dumps(dict(goal_published=details)),flush=True)
 next_report=sent;previous_controller=None
 while True:
  rclpy.spin_once(n,timeout_sec=.02)
  if (folder/'STOP').exists(): reason='ASSISTANT_REQUESTED_STOP';break
  if time.monotonic()-sent>args.max_duration_s: reason='TRIAL_TIME_LIMIT';break
  if args.map_plan: check_map_binding()
  c=status.get('go2_action_executor')
  if time.monotonic()-sent>2 and c and c.state=='INHIBITED': reason='CONTROLLER_INHIBITED:'+c.error_code;break
  # A queued arrival for an older task cannot certify this newly injected goal.
  if (pg and task and task.episode_id and pg.episode_id==task.episode_id
      and task.header.stamp.sec*10**9+task.header.stamp.nanosec>=goal_stamp_ns
      and pg.header.stamp.sec*10**9+pg.header.stamp.nanosec>=goal_stamp_ns
      and time.monotonic()-latest.get('pointgoal',(0,None))[0]<.5
      and pose_key in latest and time.monotonic()-latest[pose_key][0]<(1.5 if args.map_plan else .3)
      and math.isfinite(pg.success_distance_m) and pg.success_distance_m>0
      and math.isfinite(pg.distance_m) and 0<=pg.distance_m<=pg.success_distance_m):
   current=current_position(1.5 if args.map_plan else .3)[0].position
   if math.hypot(current.x-goal.point.x,current.y-goal.point.y)<=pg.success_distance_m:
    reason='GOAL_DISTANCE_REACHED';break
  planner=status.get(planner_name)
  if (args.navigation_mode=='direct_goal' and planner and planner.state=='TERMINAL'
      and planner.header.stamp.sec*10**9+planner.header.stamp.nanosec>=goal_stamp_ns):
   reason='PIXNAV_TERMINAL_BEFORE_GOAL';break
  if time.monotonic()>=next_report:
   current=latest[pose_key][1].pose.pose.position
   v=status.get(planner_name); px=status.get('pixnav_runtime_node')
   print(json.dumps(dict(elapsed_s=round(time.monotonic()-sent,1),goal_distance_m=pg.distance_m if pg else None,displacement_m=math.hypot(current.x-p.position.x,current.y-p.position.y),controller=dict(state=c.state,mode=c.active_mode,error=c.error_code) if c else None,vlm=dict(state=v.state,mode=v.active_mode,error=v.error_code) if v else None,pixnav=dict(state=px.state,mode=px.active_mode,error=px.error_code) if px else None,results=len(results))),flush=True);next_report=time.monotonic()+5
except KeyboardInterrupt:
 reason='OPERATOR_INTERRUPTED'
except Exception as exc:
 reason='ERROR:'+str(exc);event('error',reason);print(reason,flush=True)
finally:
 if armed:
  # Issue StopMove immediately, even if the ROS enable service is unavailable.
  # The host wrapper also stops the command-owning processes on every exit.
  try:
   from s2e_vlm_robot.sport_client import SportClient
   SportClient('/run/s2e-robot/commands.sock',timeout_s=.2).stop()
   event('direct_stop',{'sent':True})
  except Exception as exc: event('direct_stop_failure',str(exc))
  try: call(False)
  except Exception as exc: shutdown_ok=False;print('STOP_SERVICE_FAILED:'+str(exc),flush=True);event('stop_failure',str(exc))
 final=dict(reason=reason,goal_frame=frame,goal_label=args.goal_label,success_distance_m=pg.success_distance_m if pg else None,max_duration_s=args.max_duration_s,navigation_mode=args.navigation_mode,look_execution=os.environ.get('PIXNAV_LOOK_EXECUTION','physical'),goal_published=published,shutdown_service_confirmed=shutdown_ok if armed else None,goal_distance_m=pg.distance_m if pg else None,results=results,statuses={k:dict(state=v.state,mode=v.active_mode,healthy=v.is_healthy,error=v.error_code) for k,v in status.items()})
 if args.fixed_start_plan:
  final.update(localization_method='fixed_start_odometry',physical_arrival_verified=False,absolute_map_localization_verified=False)
 (folder/'result.json').write_text(json.dumps(final,indent=2));print(json.dumps(final),flush=True)
 log.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
raise SystemExit(0 if reason=='GOAL_DISTANCE_REACHED' and shutdown_ok else 1)
