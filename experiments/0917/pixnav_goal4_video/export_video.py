#!/usr/bin/env python3
"""Export the selected PixelNav Goal4/005 episode from its preserved MCAP."""
import argparse,csv,datetime,json,math,os,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
EPISODE=Path('/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-recaptured5-20260914/episodes/direct_goal-goal-4-005')
TOPIC='/robot_nav/sensors/front_camera/image_raw'
FPS=15

def bounds():
    events=[json.loads(l) for l in (EPISODE/'full5m/runtime/events.jsonl').open()]
    goal=next(x for x in events if x['event']=='goal_published')
    stop=next(x for x in events if x['event']=='direct_stop')
    ns=lambda r:round(datetime.datetime.fromisoformat(r['utc']).timestamp()*1e9)
    return goal,stop,ns(goal),ns(stop)

def decode():
    # C++/ROS reader messages must never mix with the binary frame stream.
    stream=os.fdopen(os.dup(1),'wb');os.dup2(2,1)
    import cv2,numpy as np,rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    cv2.setNumThreads(1)
    goal,stop,start_ns,stop_ns=bounds();end_ns=stop_ns+1500000000
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(EPISODE/'native-inputs'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    previous=None;previous_ns=None;previous_header=None;out_frames=0;source_frames=0;largest_gap=0;first=None;last=None
    used_sources=set();entries=[];lookahead=False
    def emit(frame,source_ns,source_header):
        nonlocal out_frames
        t_ns=start_ns+round(out_frames*1000000000/FPS)
        stream.write(frame.tobytes())
        entries.append(dict(frame_index=out_frames,video_time_s=out_frames/FPS,
            episode_time_s=(t_ns-start_ns)*1e-9,source_receipt_unix_ns=source_ns,
            source_header_ns=source_header,hold_age_s=(t_ns-source_ns)*1e-9))
        used_sources.add(source_ns)
        if out_frames in (0,30*FPS,58*FPS):cv2.imwrite(str(ROOT/('preview_%04d.jpg'%out_frames)),frame)
        out_frames+=1
    while reader.has_next():
        _,raw,t_ns=reader.read_next()
        if t_ns<start_ns-1000000000:continue
        if previous_ns is not None and t_ns<previous_ns:raise ValueError('Camera bag receipt timestamp reversal')
        m=deserialize_message(raw,Image)
        if m.width!=1280 or m.height!=720 or m.encoding not in ('rgb8','bgr8'):
            raise ValueError('Unexpected source image format')
        frame=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
        if m.encoding=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
        header=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        if previous is None:
            if t_ns>start_ns:raise ValueError('No source RGB at goal-publication boundary')
        else:
            while start_ns+round(out_frames*1000000000/FPS)<min(t_ns,end_ns):
                emit(previous,previous_ns,previous_header)
        if t_ns>=end_ns:lookahead=True;break
        if previous_ns is not None:largest_gap=max(largest_gap,t_ns-previous_ns)
        first=t_ns if first is None else first;last=t_ns;source_frames+=1
        previous=frame;previous_ns=t_ns;previous_header=header
    if not lookahead:
        while previous is not None and start_ns+round(out_frames*1000000000/FPS)<=last:
            emit(previous,previous_ns,previous_header)
    stream.close()
    if not out_frames:raise ValueError('No video frames exported')
    with (ROOT/'pixnav-frame-timeline.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=entries[0]);writer.writeheader();writer.writerows(entries)
    report=dict(source_episode=str(EPISODE),source_topic=TOPIC,source_bag=str(EPISODE/'native-inputs'),
        start_utc=goal['utc'],stop_utc=stop['utc'],source_first_receipt_unix_ns=first,source_last_receipt_unix_ns=last,
        video_zero='Goal publication; matches pixnav_trajectory.csv t_s origin',source_frames_read=source_frames,
        unique_source_frames_in_output=len(used_sources),output_frames=out_frames,output_fps=FPS,
        output_duration_s=out_frames/FPS,episode_duration_s=stop['elapsed_s']-goal['elapsed_s'],
        stop_video_time_s=(stop_ns-start_ns)*1e-9,maximum_source_receipt_gap_s=largest_gap*1e-9,
        timing='Original bag receipt timestamps, previous-frame hold at 15fps; no pauses removed and no motion interpolation',
        output_resolution=[1280,720],resize=False,crop=False,audio=False,external_robot_video=False,
        navigation_mode='direct_goal',vlm_used=False,result='PIXNAV_TERMINAL_BEFORE_GOAL',
        source_modified=False,new_hashes_computed=False)
    (ROOT/'video-provenance.json').write_text(json.dumps(report,indent=2)+'\n')

def export():
    output=ROOT/'pixnav_goal4_005_rgb_1x.mp4'
    if output.exists():raise SystemExit('Output already exists; preserve it')
    docker=['docker','run','--rm','--init','--log-driver','none','--name','pixnav-goal4-video-export','--network','none',
        '--user',f'{os.getuid()}:{os.getgid()}','-e','HOME=/tmp','-e','ROS_DOMAIN_ID=189',
        '-v','/home/unitree:/home/unitree:ro','-v',f'{ROOT}:{ROOT}',
        '--entrypoint','bash','escape-vins-fusion:jazzy-20260917','-lc',
        'source /opt/ros/jazzy/setup.bash && exec python3 "$@"','decode',str(Path(__file__).resolve()),'--decode']
    encode=['ffmpeg','-hide_banner','-loglevel','error','-n','-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r',str(FPS),
        '-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
    (ROOT/'export-command.json').write_text(json.dumps(dict(decode=docker,encode=encode),indent=2)+'\n')
    with (ROOT/'decode.log').open('x') as log:
        producer=subprocess.Popen(docker,stdout=subprocess.PIPE,stderr=log)
        encoder=subprocess.Popen(encode,stdin=producer.stdout);producer.stdout.close()
        encoded=encoder.wait();decoded=producer.wait()
    if encoded or decoded:
        subprocess.run(['docker','rm','-f','pixnav-goal4-video-export'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        raise RuntimeError(f'Export failed: decoder={decoded}, encoder={encoded}')
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-n','-i',str(output),'-vf',
         "setpts=0.25*PTS,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='Direct Goal PixelNav | Goal 4 | Run 005 | 4x':x=14:y=14:fontsize=22:fontcolor=white:box=1:boxcolor=black@0.65:boxborderw=6",
         '-r',str(FPS),'-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(ROOT/'pixnav_goal4_005_rgb_4x.mp4')]
    subprocess.run(cmd,check=True)
    print(json.dumps(json.loads((ROOT/'video-provenance.json').read_text()),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--decode',action='store_true');a=p.parse_args()
    decode() if a.decode else export()
