#!/usr/bin/env python3
"""Offline derived-copy size probe. No rclpy, ROS publication, or source writes."""
import collections
import hashlib
import json
from pathlib import Path
import struct
import time

import rosbag2_py

SOURCE = Path('/input/sample.mcap')
OUT = Path('/out/sample-essential-native')
KEEP = {
    '/s2e/robot/sensors/odom', '/s2e/robot/sensors/imu',
    '/robot_nav/sensors/front_camera/camera_info', '/tf_static', '/tf',
}


def reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    return r


def add(hashes, counts, topic, timestamp, data):
    hashes.setdefault(topic, hashlib.sha256()).update(struct.pack('<qQ', timestamp, len(data)) + data)
    counts[topic] += 1


def main():
    assert not OUT.exists(), 'Refuse to overwrite a previous sample'
    assert {p.name for p in Path('/sys/class/net').iterdir()} == {'lo'}
    started = time.monotonic()
    r = reader(SOURCE)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(OUT), storage_id='mcap', storage_preset_profile='zstd_fast'),
           rosbag2_py.ConverterOptions('', ''))
    for topic in r.get_all_topics_and_types():
        if topic.name in KEEP:
            w.create_topic(topic)
    hashes, counts = {}, collections.Counter()
    original_counts = collections.Counter()
    while r.has_next():
        topic, data, timestamp = r.read_next()
        original_counts[topic] += 1
        if topic in KEEP:
            add(hashes, counts, topic, timestamp, data)
            w.write(topic, data, timestamp)
    del w
    del r
    copied_hashes, copied_counts = {}, collections.Counter()
    check = reader(OUT)
    while check.has_next():
        topic, data, timestamp = check.read_next()
        add(copied_hashes, copied_counts, topic, timestamp, data)
    same = counts == copied_counts and {k: v.hexdigest() for k, v in hashes.items()} == {
        k: v.hexdigest() for k, v in copied_hashes.items()}
    assert same
    copied_bytes = sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    result = dict(source='/home/unitree/s2e-vlm-async-framework-minimal/.local-data/recording-five-goals-20260913/episodes/full-goal-4-009/native-inputs/native-inputs_0.mcap',
                  source_bytes=SOURCE.stat().st_size, derived_bytes=copied_bytes,
                  original_counts=dict(original_counts), retained_counts=dict(counts),
                  retained_topic_timestamp_payload_sha256={k: v.hexdigest() for k, v in hashes.items()},
                  retained_streams_exact=same, source_modified=False, network='none',
                  physical_commands=0, elapsed_s=time.monotonic()-started,
                  scope='Single bag segment derived-copy proof only. Not a validated live compact recording profile. Runtime control pose, task/commands, model inputs and operator annotations live outside this sample and must also be retained.')
    Path('/out/compact_sample_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
