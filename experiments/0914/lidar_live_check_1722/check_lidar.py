#!/usr/bin/env python3
"""Bounded, subscription-only lidar diagnosis; never publishes or calls services."""
import json
import math
import time
import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from unitree_go.msg import LidarState
from std_msgs.msg import String
from rosidl_runtime_py.convert import message_to_ordereddict

parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=40)
parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parent)
parser.add_argument('--topics', nargs='*')
args = parser.parse_args()
OUT = args.out
OUT.mkdir(parents=True, exist_ok=True)
TOPICS = {
    '/utlidar/cloud': PointCloud2,
    '/utlidar/cloud_deskewed': PointCloud2,
    '/utlidar/cloud_base': PointCloud2,
    '/utlidar/robot_odom': Odometry,
    '/utlidar/imu': Imu,
    '/utlidar/lidar_state': LidarState,
    '/utlidar/server_log': String,
}
if args.topics:
    TOPICS = {t: TOPICS[t] for t in args.topics}
records = {t: [] for t in TOPICS}
snapshots = {}
states = []
logs = []
errors = []


def callback(topic, msg):
    now = time.monotonic()
    wall = time.time()
    row = dict(received_monotonic_s=now, received_unix_s=wall)
    if hasattr(msg, 'header'):
        row['stamp_s'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row['clock_offset_plus_delay_s'] = wall - row['stamp_s']
        row['frame'] = msg.header.frame_id
    try:
        if isinstance(msg, PointCloud2):
            row.update(width=msg.width, height=msg.height, point_step=msg.point_step,
                       row_step=msg.row_step, data_bytes=len(msg.data), is_dense=msg.is_dense)
            row['fields'] = [dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in msg.fields]
            fields = {f.name: f for f in msg.fields}
            endian = '>' if msg.is_bigendian else '<'
            if any(k not in fields or fields[k].datatype != 7 or fields[k].count != 1 for k in ('x', 'y', 'z')):
                raise ValueError('Unsupported XYZ field layout')
            if len(msg.data) < msg.row_step * msg.height:
                raise ValueError('Short PointCloud2 data buffer')
            dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [endian+'f4']*3,
                              'offsets': [fields[k].offset for k in ('x', 'y', 'z')], 'itemsize': msg.point_step})
            arr = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                             strides=(msg.row_step, msg.point_step))
            xyz = np.stack([arr[k].ravel() for k in ('x', 'y', 'z')], axis=1)
            finite = np.isfinite(xyz).all(axis=1)
            zeros = (xyz == 0).all(axis=1)
            valid = xyz[finite & ~zeros]
            row.update(points=len(xyz), finite_points=int(finite.sum()),
                       zero_xyz_points=int(zeros.sum()), valid_points=len(valid))
            if len(valid):
                ranges = np.linalg.norm(valid, axis=1)
                row['range_m_percentiles_0_5_50_95_100'] = np.percentile(ranges, [0, 5, 50, 95, 100]).tolist()
                row['xyz_min'] = valid.min(axis=0).tolist()
                row['xyz_max'] = valid.max(axis=0).tolist()
                row['azimuth_12bins'] = np.histogram(np.arctan2(valid[:, 1], valid[:, 0]), bins=12, range=(-math.pi, math.pi))[0].tolist()
            n = len(records[topic])
            if n in (0, 150, 300):
                snapshots[topic.strip('/').replace('/', '_')+'_'+str(n)] = xyz
            snapshots[topic.strip('/').replace('/', '_')+'_last'] = xyz
        elif isinstance(msg, Odometry):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            row.update(position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
                       linear_velocity=[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
                       angular_velocity=[msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z],
                       child_frame=msg.child_frame_id)
        elif isinstance(msg, Imu):
            row.update(gyro=[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                       acceleration=[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        elif isinstance(msg, LidarState):
            state = dict(message_to_ordereddict(msg))
            states.append(dict(received_unix_s=wall, **state))
        elif isinstance(msg, String):
            logs.append(dict(received_unix_s=wall, text=msg.data[:3000]))
    except Exception as exc:
        row['decode_error'] = str(exc)
        errors.append(dict(topic=topic, error=str(exc)))
    row['callback_elapsed_ms'] = (time.monotonic() - now)*1000
    records[topic].append(row)


def stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return dict(min=float(values.min()), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)), max=float(values.max()))


rclpy.init()
node = rclpy.create_node('readonly_lidar_diagnostic')
subscriptions = [node.create_subscription(cls, topic, lambda msg, topic=topic: callback(topic, msg), qos_profile_sensor_data)
                 for topic, cls in TOPICS.items()]
started = time.monotonic()
started_unix = time.time()
next_print = started + 10
while time.monotonic() - started < args.seconds:
    rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() >= next_print:
        print(json.dumps({'elapsed_s': round(time.monotonic()-started, 1),
                          'received': {t: len(rows) for t, rows in records.items()},
                          'latest_state': states[-1] if states else None}), flush=True)
        next_print += 10
graph = {name: types for name, types in node.get_topic_names_and_types()}
node.destroy_node()
rclpy.shutdown()
summary = dict(started_unix_s=started_unix, duration_s=time.monotonic()-started,
               note='Subscription-only, no commands/services. Raw wall-header difference includes clock offset; not absolute sensor latency.',
               topics={}, decode_errors=errors, lidar_states=states, server_logs=logs, ros_graph=graph)
for topic, rows in records.items():
    item = dict(count=len(rows))
    if rows:
        received = np.array([r['received_monotonic_s'] for r in rows])
        gaps = np.diff(received)
        item.update(receive_interval_ms=stats(gaps*1000), callback_elapsed_ms=stats([r['callback_elapsed_ms'] for r in rows]))
        item['receive_hz'] = float((len(rows)-1)/(received[-1]-received[0])) if len(rows)>1 else None
        if 'stamp_s' in rows[0]:
            stamps = np.array([r['stamp_s'] for r in rows])
            source_gaps = np.diff(stamps)
            offsets = np.array([r['clock_offset_plus_delay_s'] for r in rows])
            item.update(source_interval_ms=stats(source_gaps*1000), duplicate_stamps=int((source_gaps==0).sum()),
                        backwards_stamps=int((source_gaps<0).sum()), clock_offset_plus_delay_s=stats(offsets),
                        offset_change_last_minus_first_s=float(offsets[-1]-offsets[0]),
                        delay_above_min_ms=stats((offsets-offsets.min())*1000), frames=sorted(set(r['frame'] for r in rows)))
        if 'points' in rows[0]:
            item.update(points=stats([r['points'] for r in rows]), valid_points=stats([r['valid_points'] for r in rows]),
                        all_invalid_frames=sum(r['valid_points']==0 for r in rows),
                        zero_xyz_fraction=stats([r['zero_xyz_points']/max(1,r['points']) for r in rows]),
                        nonfinite_fraction=stats([1-r['finite_points']/max(1,r['points']) for r in rows]),
                        first=rows[0], last=rows[-1])
        if 'position' in rows[0]:
            positions = np.array([r['position'] for r in rows])
            item.update(first_position=positions[0].tolist(), last_position=positions[-1].tolist(),
                        displacement_m=float(np.linalg.norm(positions[-1]-positions[0])),
                        xyz_span_m=np.ptp(positions, axis=0).tolist(),
                        largest_consecutive_position_step_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).max()) if len(rows)>1 else None)
        if 'gyro' in rows[0]:
            item['gyro_norm_rad_s'] = stats(np.linalg.norm([r['gyro'] for r in rows], axis=1))
            item['acceleration_norm_m_s2'] = stats(np.linalg.norm([r['acceleration'] for r in rows], axis=1))
    summary['topics'][topic] = item
(OUT/'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
(OUT/'samples.json').write_text(json.dumps(records, ensure_ascii=False))
np.savez_compressed(OUT/'cloud_snapshots.npz', **snapshots)
print(json.dumps({'summary': str(OUT/'summary.json'), 'topics': {t: {k: v for k, v in s.items() if k not in ('first','last')} for t,s in summary['topics'].items()}, 'latest_state': states[-1] if states else None}, ensure_ascii=False), flush=True)
