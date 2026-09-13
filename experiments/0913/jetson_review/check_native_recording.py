"""File-only synthetic recorder check: run only with network none and domain 214."""
import json,os,signal,subprocess,time
from pathlib import Path
assert set(os.listdir('/sys/class/net'))=={'lo'}
assert os.environ['ROS_DOMAIN_ID']=='214'
import rclpy,rosbag2_py,yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField,Image,Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import struct
service=yaml.safe_load(Path('/work/.local-data/jetson-post-drive-20260913/prepared-recorded-pairs/full_goal_1/compose.trial.yaml').read_text())['services']['native-recording']
assert 'bag record' in service['command'][0]
p=subprocess.Popen(['bash','-lc',service['command'][0]],stdout=open('/trial/recorder-console.log','w'),stderr=subprocess.STDOUT)
rclpy.init();n=rclpy.create_node('synthetic_recording_check')
pubs={t:n.create_publisher(cls,t,qos_profile_sensor_data) for t,cls in [('/s2e/mapping/cloud',PointCloud2),('/s2e/robot/sensors/cloud',PointCloud2),('/s2e/robot/sensors/odom',Odometry),('/s2e/robot/sensors/imu',Imu),('/robot_nav/sensors/front_camera/image_raw',Image)]}
try:
 time.sleep(1)
 for i in range(100):
  assert p.poll() is None,'Recorder exited early'
  stamp=n.get_clock().now().to_msg()
  cloud=PointCloud2(header=Header(stamp=stamp,frame_id='base_link'),height=1,width=1,fields=[PointField(name='x',offset=0,datatype=7,count=1),PointField(name='y',offset=4,datatype=7,count=1),PointField(name='z',offset=8,datatype=7,count=1)],is_bigendian=False,point_step=12,row_step=12,data=struct.pack('<fff',1.,.2,.3),is_dense=True)
  pubs['/s2e/mapping/cloud' if i<40 else '/s2e/robot/sensors/cloud'].publish(cloud)
  od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.pose.pose.orientation.w=1.
  pubs['/s2e/robot/sensors/odom'].publish(od)
  im=Imu();im.header.stamp=stamp;pubs['/s2e/robot/sensors/imu'].publish(im)
  rgb=Image(header=Header(stamp=stamp,frame_id='camera'),height=2,width=2,encoding='rgb8',step=6,data=bytes([16]*12))
  pubs['/robot_nav/sensors/front_camera/image_raw'].publish(rgb)
  rclpy.spin_once(n,timeout_sec=.01);time.sleep(.04)
finally:
 p.send_signal(signal.SIGINT);code=p.wait(timeout=20)
 n.destroy_node();rclpy.shutdown()
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri='/trial/native-inputs',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
counts={}
while reader.has_next():
 topic,_,_=reader.read_next();counts[topic]=counts.get(topic,0)+1
assert all(counts.get(t,0)>0 for t in pubs),counts
result=dict(scope='Synthetic messages, isolated network; validates recorder and ownership-topic handover only',recorder_exit_code=code,counts=counts,passed=True,physical_inputs_used=False)
Path('/trial/result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
