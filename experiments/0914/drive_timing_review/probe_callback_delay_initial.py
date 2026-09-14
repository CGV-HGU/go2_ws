"""Isolated ROS scheduling probe; substitutes all Sport transport methods.

Fresh synthetic odom/pose/RGB headers arrive while synchronous command calls
take controlled time. Not physical sensor/load/firmware validation.
"""
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from s2e_vlm_msgs.msg import StampedPose,NodeStatus
from s2e_vlm_robot import action_executor_node as module
from s2e_vlm_robot.motion import MotionOutput

assert {p.name for p in Path('/sys/class/net').iterdir()}=={'lo'}


class RecordingTransport:
    def __init__(self,delay):self.delay=delay;self.moves=0;self.stops=0
    def move(self,*args):self.moves+=1;time.sleep(self.delay)
    def stop(self):self.stops+=1
    def ping(self):return 1


class ProbeMotion:
    kind='translate';target=100.;heading_yaw=0.;progress=0.
    def __init__(self):self.started=time.monotonic()
    def step(self,sample,**kwargs):
        self.progress=sample.x
        return MotionOutput(linear=.1,progress=self.progress)
    def finish(self,reason):return MotionOutput(terminal=reason,progress=self.progress)


def run(delay):
    rclpy.init()
    transport=RecordingTransport(delay)
    module.SportClient=lambda *args:transport
    executor_node=module.ActionExecutorNode();io=Node('isolated_callback_probe')
    executor_node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    pubodom=io.create_publisher(Odometry,'/s2e/robot/sensors/odom',qos_profile_sensor_data)
    pubpose=io.create_publisher(StampedPose,'/s2e/odometry/pose',qos_profile_sensor_data)
    pubrgb=io.create_publisher(Image,'/s2e/sensors/camera/image',qos_profile_sensor_data)
    health=[(name,io.create_publisher(NodeStatus,topic,20)) for name,topic in [
       ('vlm_node','/s2e/status/vlm_node'),('pixnav_runtime_node','/s2e/status/pixnav_runtime_node'),
       ('sweep_scheduler_node','/s2e/status/sweep_scheduler_node'),('rtab_pointgoal_adapter','/s2e/robot/observation_status')]]
    latest_observed_pose=[]
    io.create_subscription(StampedPose,'/s2e/odometry/pose',
        lambda m:latest_observed_pose.append((time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),qos_profile_sensor_data)
    ex=MultiThreadedExecutor(num_threads=3);ex.add_node(executor_node);ex.add_node(io)
    spin=threading.Thread(target=ex.spin,daemon=True);spin.start()
    stop=threading.Event();start=time.monotonic();counts={'frames':0}
    def publish():
        while not stop.is_set():
            stamp=io.get_clock().now().to_msg();x=(time.monotonic()-start)*.1
            od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.position.x=x;od.pose.pose.orientation.w=1.;od.twist.twist.linear.x=.1
            pose=StampedPose();pose.header.stamp=stamp;pose.header.frame_id='robot_origin';pose.pose.position.x=x;pose.pose.orientation.w=1.;pose.status='OK';pose.child_frame_id='base_link'
            im=Image();im.header.stamp=stamp;im.header.frame_id='camera';im.width=1;im.height=1;im.step=3;im.encoding='rgb8';im.data=[0,0,0]
            pubodom.publish(od);pubpose.publish(pose);pubrgb.publish(im)
            for name,pub in health:
                h=NodeStatus();h.header.stamp=stamp;h.node_name=name;h.is_healthy=True;h.state='READY';pub.publish(h)
            counts['frames']+=1;stop.wait(1/30)
    sender=threading.Thread(target=publish,daemon=True);sender.start()
    samples=[];original=executor_node._admission_error
    def admission(now):
        result=original(now)
        stamp=executor_node._last_pose.header.stamp if executor_node._last_pose else None
        age=(executor_node.get_clock().now().nanoseconds/1e9-stamp.sec-stamp.nanosec*1e-9) if stamp else None
        independent=(time.time()-latest_observed_pose[-1][1]) if latest_observed_pose else None
        samples.append(dict(t=time.monotonic()-start,error=result,pose_age_s=age,independent_pose_age_s=independent))
        return result
    executor_node._admission_error=admission
    try:
        deadline=time.monotonic()+3
        while original(time.monotonic()):
            assert time.monotonic()<deadline,'fixture did not become ready'
            time.sleep(.025)
        response=executor_node._set_motion_enabled(SimpleNamespace(data=True),SimpleNamespace())
        assert response.success,response.message
        with executor_node._lock:executor_node._active=ProbeMotion()
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and executor_node._motion_enabled:time.sleep(.05)
        result=dict(delay_s=delay,mock_moves=transport.moves,fault=executor_node._last_fault,
                    maximum_pose_age_s=max((s['pose_age_s'] or 0) for s in samples),
                    sample_count=len(samples),independent_pose_samples=len(latest_observed_pose),samples=samples)
    finally:
        executor_node._set_motion_enabled(SimpleNamespace(data=False),SimpleNamespace())
        stop.set();sender.join(2);ex.shutdown(timeout_sec=2);spin.join(2)
        executor_node.destroy_node();io.destroy_node();rclpy.shutdown()
    return result


results=[]
for delay in [0,.02,.08]:
    result=run(delay);results.append(result)
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
Path('/out/callback_delay_results.json').write_text(json.dumps(dict(cases=results,physical_commands=0,
 scope='Installed ROS callbacks, synthetic 30Hz headers, 3 executor threads and recording transport. Tests scheduling exposure only; no GPU/VLM/bag full load or actual Sport network.'),indent=2)+'\n')
