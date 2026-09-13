#!/usr/bin/env python3
"""Prepare/manage a mapping-to-localization session with continuous sensor input.

Only 'prepare' is offline. The other commands start/stop sensor and SLAM nodes;
none starts a navigation controller or publishes a motion/PointGoal command.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sqlite3
import subprocess
import time
import yaml

REPO=Path(__file__).resolve().parents[1]
NAV_IMAGE='escape-navigation:map-control-20260913'
SLAM_IMAGE='sha256:478e26f81e6d344e691215508b9409939df53ed46bf57f5bcfd1d04356ca0d28'


def write_json(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def planar_pose(p):
    q=p['orientation'];v=p['position']
    if not all(math.isfinite(x) for x in (*q.values(),*v.values())):
        raise ValueError('Invalid initial pose')
    if abs(sum(q[k]**2 for k in ('x','y','z','w'))-1.)>.01:
        raise ValueError('Invalid initial orientation')
    return v['x'],v['y'],v['z'],math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))


def continued_initial_pose(before,after,boot_id,now):
    """Carry map<-odom across a SLAM restart, never across an odometry reset."""
    if before.get('host_boot_id')!=boot_id or after.get('host_boot_id')!=boot_id:
        raise ValueError('Cannot carry a pose across a Jetson reboot')
    gids=before.get('odom_publisher_gids',[])
    if len(gids)!=1 or gids!=after.get('odom_publisher_gids'):
        raise ValueError('Odometry publisher changed or has multiple owners')
    if not 0<=now-after['checked_unix']<=2:
        raise ValueError('Current odometry snapshot is stale')
    a=before['topics']['odom'];b=after['topics']['odom']
    if a['frame']!='odom' or b['frame']!='odom' or b['stamp_unix']<a['stamp_unix']:
        raise ValueError('Odometry frame or time changed')
    if not -.1<=now-b['stamp_unix']<=1. or not 0<=now-b['received_unix']<=1.:
        raise ValueError('Current odometry is stale')
    old=planar_pose(a['pose']['pose']);cur=planar_pose(b['pose']['pose'])
    angle=math.atan2(math.sin(cur[3]-old[3]),math.cos(cur[3]-old[3]))
    if math.hypot(cur[0]-old[0],cur[1]-old[1])>.05 or abs(angle)>.05:
        raise ValueError('Keep the robot stationary during map freeze/localization startup')
    transform=before['map_to_odom']
    if transform['header']['frame_id']!='map' or transform['child_frame_id']!='odom':
        raise ValueError('Expected map<-odom transform')
    t=transform['transform'];tx,ty,tz,yaw=planar_pose(dict(position=t['translation'],orientation=t['rotation']))
    c,s=math.cos(yaw),math.sin(yaw)
    result=[tx+c*cur[0]-s*cur[1],ty+s*cur[0]+c*cur[1],tz+cur[2],0.,0.,math.atan2(math.sin(yaw+cur[3]),math.cos(yaw+cur[3]))]
    return ' '.join(format(v,'.12g') for v in result)


def known_start_localization(config, initial_pose):
    """Track from a verified carried pose; this is not global relocalization.

    Native 0.22.1 replay on 2026-09-13 showed identity-seeded global ICP
    erasing a real observation rotation. Seed one reference scan from odom
    and do not fall back to that unseeded path. Never merge nearby scans.
    """
    values=[float(v) for v in initial_pose.split()]
    if len(values)!=6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Known-start tracking requires a finite six-value initial pose')
    result=copy.deepcopy(config)
    p=result['/**']['ros__parameters']
    p.update({
        'initial_pose':initial_pose,
        'Mem/IncrementalMemory':'false',
        'RGBD/ProximityBySpace':'true',
        'RGBD/ProximityOdomGuess':'true',
        'RGBD/ProximityPathMaxNeighbors':'0',
        'RGBD/ProximityMaxPaths':'1',
        'RGBD/ProximityAngle':'180',
        'RGBD/NeighborLinkRefining':'false',
        'Rtabmap/LoopThr':'1',
        'RGBD/AggressiveLoopThr':'1',
        'RGBD/LocalizationSecondTryWithoutProximityLinks':'false',
    })
    return result


def prepare(run,navigation_image):
    run=Path(run).resolve();run.mkdir(parents=True,exist_ok=False)
    (run/'config').mkdir();(run/'ros-log').mkdir()
    shutil.copy2(REPO/'config/robot-localization.yaml',run/'config/localization.yaml')
    mapping=yaml.safe_load((REPO/'config/robot-mapping.yaml').read_text())
    mapping['/**']['ros__parameters']['database_path']='/localization/mapping.db'
    (run/'config/mapping.yaml').write_text(yaml.safe_dump(mapping,sort_keys=False))
    shutil.copy2(REPO/'scripts/robot_localization_recorder.py',run/'recorder.py')
    shutil.copy2(REPO/'.local-data/prepared-full-20260912-ordered-pose/run_001/config/cyclonedds.xml' if (REPO/'.local-data/prepared-full-20260912-ordered-pose/run_001/config/cyclonedds.xml').exists() else Path('/home/unitree/go2_ws_antarctica/cyclonedds.xml'),run/'config/cyclonedds.xml')
    common=dict(network_mode='host',init=True,restart='no',stop_signal='SIGINT',stop_grace_period='45s',
                environment=dict(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file:///localization/config/cyclonedds.xml',ROS_LOG_DIR='/localization/ros-log'),
                volumes=[str(run)+':/localization'],entrypoint=['bash','-lc'])
    services={}
    setup='source /opt/ros/jazzy/setup.bash && '
    for name,command in [('sensors','ros2 run s2e_vlm_robot go2_sensor_input'),('input','python3 -m s2e_vlm_robot.mapping_input_node')]:
        services[name]=dict(copy.deepcopy(common),image=navigation_image,command=[setup+'source /opt/s2e-robot-minimal/setup.bash && exec '+command])
    services['recorder']=dict(copy.deepcopy(common),image=SLAM_IMAGE,command=[setup+'exec python3 /localization/recorder.py'])
    # Preserve actual timestamped inputs for later native RTAB-Map replay.
    # Existing JSON/thumbnail recorders cannot reconstruct moving ICP inputs.
    services['native-recording']=dict(copy.deepcopy(common),image=SLAM_IMAGE,cpus=2.,command=[
        setup+'exec ros2 bag record --storage mcap --storage-preset-profile zstd_fast '
        '--disable-keyboard-controls --max-bag-size 536870912 '
        '--output /localization/native-inputs --topics '
        '/robot_nav/sensors/front_camera/image_raw /robot_nav/sensors/front_camera/camera_info '
        '/s2e/mapping/cloud /s2e/robot/sensors/cloud /s2e/robot/sensors/odom /s2e/robot/sensors/imu '
        '/tf /tf_static /rtabmap/localization_pose /rtabmap/info'])
    remap=' -r __ns:=/rtabmap -r __node:=rtabmap -r rgb/image:=/robot_nav/sensors/front_camera/image_raw -r rgb/camera_info:=/robot_nav/sensors/front_camera/camera_info -r odom:=/s2e/robot/sensors/odom -r scan_cloud:=/s2e/mapping/cloud -r imu:=/s2e/robot/sensors/imu'
    services['slam']=dict(copy.deepcopy(common),image=SLAM_IMAGE,command=[setup+'exec /opt/ros/jazzy/lib/rtabmap_slam/rtabmap --ros-args --params-file /localization/config/mapping.yaml'+remap])
    compose=dict(name='escape-map-session',services=services)
    write_json(run/'compose.json',compose)
    compose['services']['slam']['command'][0]=compose['services']['slam']['command'][0].replace('/config/mapping.yaml','/config/localization.yaml')
    write_json(run/'compose.localization.json',compose)
    manifest=dict(path=str(run),status='prepared',navigation_image=navigation_image,slam_image=SLAM_IMAGE,source_database=None,source_database_sha256=None,host_boot_id=None,physical_localization_verified=False,
                  native_input_archive=str(run/'native-inputs'))
    manifest['prepared_sha256']={str(p.relative_to(run)):digest(p) for p in sorted(run.rglob('*')) if p.is_file()}
    write_json(run/'manifest.json',manifest)
    return manifest


def command(run,filename,*args):
    return subprocess.run(['docker','compose','-f',str(run/filename),*args],check=True)


def set_state(run,manifest,status):
    manifest=dict(manifest,status=status,updated_unix=time.time())
    write_json(run/'manifest.json',manifest)
    state=REPO/'.local-data/current-localization.json';state.parent.mkdir(exist_ok=True)
    write_json(state,manifest)
    return manifest


def freeze_fixed_start(run, manifest, boot):
    """Freeze the map, keep the odometry recorder for manual goal collection."""
    if manifest['status'] != 'mapping' or manifest['host_boot_id'] != boot:
        raise ValueError('Require this boot\'s mapping session')
    if (run/'map.db').exists():
        raise ValueError('Frozen map already exists')
    command(run,'compose.json','stop','slam')
    ids=subprocess.check_output(['docker','compose','-f',str(run/'compose.json'),'ps','-a','-q','slam'],text=True).strip()
    state=json.loads(subprocess.check_output(['docker','inspect',ids]))[0]['State']
    if state['Running'] or state['ExitCode'] != 0:
        raise ValueError('Mapping SLAM did not exit cleanly')
    # SQLite backup also preserves any committed WAL content.
    with sqlite3.connect('file:'+str(run/'mapping.db')+'?mode=ro',uri=True) as source:
        if source.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
            raise ValueError('Map integrity check failed')
        if source.execute('SELECT count(*) FROM Node').fetchone()[0] == 0:
            raise ValueError('Map has no nodes')
        with sqlite3.connect(str(run/'map.db')) as dest:
            source.backup(dest)
    manifest.update(source_database=str(run/'map.db'),source_database_sha256=digest(run/'map.db'),
                    localization_mode='fixed_start_odometry',physical_localization_verified=False,
                    arbitrary_start_relocalization_verified=False)
    return set_state(run,manifest,'collecting_fixed_start_goals')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('prepare','start-mapping','freeze-localize','freeze-fixed-start','stop'))
    p.add_argument('--run-dir',type=Path,required=True)
    args=p.parse_args();run=args.run_dir.resolve()
    if args.action=='prepare':
        image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}',NAV_IMAGE],text=True).strip()
        print(json.dumps(prepare(run,image),indent=2));return
    manifest=json.loads((run/'manifest.json').read_text())
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if args.action=='start-mapping':
        if manifest['status']!='prepared':raise ValueError('Use a new prepared session')
        for name,sha in manifest['prepared_sha256'].items():
            if digest(run/name)!=sha:raise ValueError('Prepared file changed: '+name)
        current=REPO/'.local-data/current-localization.json'
        if current.exists() and json.loads(current.read_text()).get('status') in ('running','mapping','starting','collecting_fixed_start_goals'):
            raise ValueError('Another map/localization session is active')
        manifest['host_boot_id']=boot
        set_state(run,manifest,'starting')
        try:
            command(run,'compose.json','up','-d','sensors','input','recorder','native-recording','slam')
            set_state(run,manifest,'mapping')
        except Exception:
            set_state(run,manifest,'failed');raise
    elif args.action=='freeze-fixed-start':
        print(json.dumps(freeze_fixed_start(run,manifest,boot),indent=2))
    elif args.action=='freeze-localize':
        if manifest['status']!='mapping' or manifest['host_boot_id']!=boot:raise ValueError('Require this boot\'s continuing mapping session')
        before=json.loads((run/'latest.json').read_text());now=time.time()
        if not 0<=now-before['checked_unix']<=2 or not before.get('map_to_odom'):raise ValueError('No current mapping pose')
        tf=before['map_to_odom'];stamp=tf['header']['stamp']['sec']+tf['header']['stamp']['nanosec']*1e-9
        if not -.3<=now-stamp<=2:raise ValueError('Mapping transform is stale')
        command(run,'compose.json','stop','slam')
        ids=subprocess.check_output(['docker','compose','-f',str(run/'compose.json'),'ps','-a','-q','slam'],text=True).strip()
        state=json.loads(subprocess.check_output(['docker','inspect',ids]))[0]['State']
        if state['Running'] or state['ExitCode']!=0:raise ValueError('Mapping SLAM did not exit cleanly')
        with sqlite3.connect('file:'+str(run/'mapping.db')+'?mode=ro',uri=True) as db:
            if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('Map integrity check failed')
        for name in ('map.db','rtabmap.db'):
            if (run/name).exists():raise ValueError('Frozen map already exists')
            shutil.copy2(run/'mapping.db',run/name)
        after=json.loads((run/'latest.json').read_text())
        initial=continued_initial_pose(before,after,boot,time.time())
        cfg=yaml.safe_load((run/'config/localization.yaml').read_text())
        cfg=known_start_localization(cfg,initial)
        (run/'config/localization.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
        manifest.update(source_database=str(run/'map.db'),source_database_sha256=digest(run/'map.db'),initial_pose=initial,
                        localization_mode='known_start_odometry_seeded_single_scan',
                        arbitrary_start_relocalization_verified=False)
        write_json(run/'initial-pose-evidence.json',dict(before=before,after=after,initial_pose=initial,ground_truth=False))
        set_state(run,manifest,'starting')
        started=time.time()
        command(run,'compose.localization.json','up','-d','--no-deps','slam')
        from capture_robot_goal import localized_goal
        deadline=time.monotonic()+45;error='No accepted map registration'
        while time.monotonic()<deadline:
            try:
                result=localized_goal(json.loads((run/'latest.json').read_text()),manifest,time.time(),requested_unix=started)
                write_json(run/'startup-registration.json',result)
                set_state(run,manifest,'running');print('Fresh map registration received; verify physical position before capturing goals.');break
            except (KeyError,ValueError) as exc:error=str(exc)
            time.sleep(.2)
        else:
            set_state(run,manifest,'localization_failed')
            raise RuntimeError(error)
    else:
        command(run,'compose.localization.json','stop','slam','recorder','native-recording','input','sensors')
        set_state(run,manifest,'stopped')


if __name__=='__main__':main()
