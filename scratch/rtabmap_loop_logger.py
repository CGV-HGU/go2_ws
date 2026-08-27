#!/usr/bin/env python3
"""Persist RTAB-Map loop-closure events for headless mapping runs.

The node subscribes only to ``rtabmap_msgs/msg/Info`` and has no publishers,
services or actuation path.  It writes both machine-readable JSON Lines and a
compact human-readable log.  Accepted/rejected events are flushed and fsynced
immediately so useful evidence survives an interrupted headless run.
"""

import json
import os
import signal
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info


def _iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')


def _safe_label(value: str) -> str:
    cleaned = ''.join(c if c.isalnum() or c in ('-', '_') else '_' for c in value.strip())
    return cleaned or 'mapping'


class RtabmapLoopLogger(Node):
    def __init__(self) -> None:
        super().__init__('rtabmap_loop_logger')
        self.declare_parameter('info_topic', '/info')
        self.declare_parameter('output_dir', '/home/unitree/.ros/rtabmap_loop_logs')
        self.declare_parameter('run_label', 'mapping')
        self.declare_parameter('heartbeat_period', 30.0)

        info_topic = str(self.get_parameter('info_topic').value)
        output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        run_label = _safe_label(str(self.get_parameter('run_label').value))
        heartbeat_period = max(5.0, float(self.get_parameter('heartbeat_period').value))

        output_dir.mkdir(parents=True, exist_ok=True)
        run_stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
        stem = f'loop_events_{run_stamp}_{run_label}'
        self.jsonl_path = output_dir / f'{stem}.jsonl'
        self.text_path = output_dir / f'{stem}.log'
        self._jsonl = self.jsonl_path.open('a', encoding='utf-8', buffering=1)
        self._text = self.text_path.open('a', encoding='utf-8', buffering=1)

        self.frames = 0
        self.accepted_global = 0
        self.accepted_proximity = 0
        self.rejected = 0
        self.highest_score = 0.0
        self.highest_candidate = 0
        self.last_ref_id = -1
        self._closed = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Info, info_topic, self._info_callback, qos)
        self.create_timer(heartbeat_period, self._heartbeat)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._write_event(
            'START',
            {
                'info_topic': info_topic,
                'jsonl_path': str(self.jsonl_path),
                'text_path': str(self.text_path),
            },
            durable=True,
        )
        self.get_logger().info(
            'Headless loop logger active: topic=%s jsonl=%s text=%s'
            % (info_topic, self.jsonl_path, self.text_path)
        )

    @staticmethod
    def _stats(msg: Info) -> Dict[str, float]:
        return {
            key: float(value)
            for key, value in zip(msg.stats_keys, msg.stats_values)
        }

    @staticmethod
    def _first_stat(stats: Dict[str, float], keys: Iterable[str], default: float = 0.0) -> float:
        for key in keys:
            base = key.rstrip('/')
            for candidate in (key, base, base + '/'):
                if candidate in stats:
                    return stats[candidate]
        return default

    @staticmethod
    def _event_stats(stats: Dict[str, float]) -> Dict[str, float]:
        prefixes = ('Loop/', 'Icp/', 'NeighborLinkRefining/')
        return {key: value for key, value in stats.items() if key.startswith(prefixes)}

    @staticmethod
    def _transform(msg: Info) -> Dict[str, Dict[str, float]]:
        t = msg.loop_closure_transform.translation
        q = msg.loop_closure_transform.rotation
        return {
            'translation': {'x': t.x, 'y': t.y, 'z': t.z},
            'rotation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
        }

    def _write_event(self, event: str, fields: Dict, durable: bool = False) -> None:
        record = {'wall_time': _iso_now(), 'event': event}
        record.update(fields)
        self._jsonl.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + '\n')

        parts = [record['wall_time'], event]
        for key in ('ref_id', 'matched_id', 'candidate_id', 'score', 'frames'):
            if key in fields:
                parts.append(f'{key}={fields[key]}')
        if event in ('HEARTBEAT', 'SUMMARY'):
            parts.extend(
                [
                    f"global={fields.get('accepted_global', self.accepted_global)}",
                    f"proximity={fields.get('accepted_proximity', self.accepted_proximity)}",
                    f"rejected={fields.get('rejected', self.rejected)}",
                    'best=%s/%.4f'
                    % (
                        fields.get('highest_candidate_id', self.highest_candidate),
                        fields.get('highest_candidate_score', self.highest_score),
                    ),
                ]
            )
        self._text.write(' '.join(parts) + '\n')

        self._jsonl.flush()
        self._text.flush()
        if durable:
            os.fsync(self._jsonl.fileno())
            os.fsync(self._text.fileno())

    def _info_callback(self, msg: Info) -> None:
        # RTAB-Map publishes at most one Info message per processed node. Guard
        # against a transient DDS replay without dropping legitimate new IDs.
        if msg.ref_id == self.last_ref_id:
            return
        self.last_ref_id = int(msg.ref_id)
        self.frames += 1

        stats = self._stats(msg)
        candidate_id = int(
            self._first_stat(
                stats,
                ('Loop/Highest_hypothesis_id', 'Loop/HighestHypothesisId'),
            )
        )
        candidate_score = self._first_stat(
            stats,
            ('Loop/Highest_hypothesis_value', 'Loop/HighestHypothesisValue'),
        )
        if candidate_score > self.highest_score:
            self.highest_score = candidate_score
            self.highest_candidate = candidate_id

        common = {
            'ros_stamp_ns': int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            'ref_id': int(msg.ref_id),
            'candidate_id': candidate_id,
            'score': candidate_score,
            'transform': self._transform(msg),
            'statistics': self._event_stats(stats),
        }

        if msg.loop_closure_id > 0:
            self.accepted_global += 1
            event = dict(common, matched_id=int(msg.loop_closure_id))
            self._write_event('ACCEPTED_GLOBAL', event, durable=True)
            self.get_logger().info(
                'LOOP ACCEPTED [global]: current=%d matched=%d score=%.4f total=%d'
                % (msg.ref_id, msg.loop_closure_id, candidate_score, self.accepted_global)
            )

        if msg.proximity_detection_id > 0:
            self.accepted_proximity += 1
            event = dict(common, matched_id=int(msg.proximity_detection_id))
            self._write_event('ACCEPTED_PROXIMITY', event, durable=True)
            self.get_logger().info(
                'LOOP ACCEPTED [proximity]: current=%d matched=%d total=%d'
                % (msg.ref_id, msg.proximity_detection_id, self.accepted_proximity)
            )

        rejected = self._first_stat(
            stats,
            ('Loop/RejectedHypothesis', 'Loop/Rejected_hypothesis'),
        )
        if rejected > 0.0 and msg.loop_closure_id <= 0 and msg.proximity_detection_id <= 0:
            self.rejected += 1
            self._write_event('REJECTED', common, durable=True)
            self.get_logger().warn(
                'LOOP REJECTED: current=%d candidate=%d score=%.4f total=%d'
                % (msg.ref_id, candidate_id, candidate_score, self.rejected)
            )

    def _summary_fields(self) -> Dict:
        return {
            'frames': self.frames,
            'accepted_global': self.accepted_global,
            'accepted_proximity': self.accepted_proximity,
            'rejected': self.rejected,
            'highest_candidate_id': self.highest_candidate,
            'highest_candidate_score': self.highest_score,
        }

    def _heartbeat(self) -> None:
        fields = self._summary_fields()
        self._write_event('HEARTBEAT', fields)
        self.get_logger().info(
            'loop status: frames=%d global=%d proximity=%d rejected=%d best=%d/%.4f'
            % (
                self.frames,
                self.accepted_global,
                self.accepted_proximity,
                self.rejected,
                self.highest_candidate,
                self.highest_score,
            )
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._write_event('SUMMARY', self._summary_fields(), durable=True)
        self._jsonl.close()
        self._text.close()

    def _signal_handler(self, _signum, _frame) -> None:
        # A foreground Ctrl+C can reach both the parent shell and this child.
        # Persist SUMMARY before asking rclpy.spin() to return.
        self.close()
        if rclpy.ok():
            rclpy.shutdown()


def main(args: Optional[Iterable[str]] = None) -> None:
    rclpy.init(args=args)
    node = RtabmapLoopLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
