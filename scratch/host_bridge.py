#!/usr/bin/env python3
"""
Host Bridge (ROS 2 Foxy / Python 3.8 on Ubuntu 20.04)
========================================================================================
- Subscribes to 50Hz RTAB-Map LIVO Odometry (/rtabmap/odom or /utlidar/robot_pose).
- Packs 7 double floats (x, y, z, qx, qy, qz, qw) with 4-byte Magic Header ('S2E\\x01')
  and checksum -> Sends 60-byte payload over UDP to Docker (127.0.0.1:9091).
- Receives 52-byte velocity command with Magic Header from Docker (127.0.0.1:9090).
- Publishes verified Twist to /cmd_vel for Unitree Go2 actuation.
"""

import socket
import struct
import zlib
import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

try:
    from unitree_api.msg import Request
    HAS_UNITREE_API = True
except ImportError:
    HAS_UNITREE_API = False

MAGIC_HEADER = 0x53324501 # 'S2E\x01'
MAX_VX = 0.35  # m/s safety clamp
MAX_WZ = 0.60  # rad/s safety clamp
WATCHDOG_TIMEOUT_S = 0.5

class HostBridge(Node):
    def __init__(self):
        super().__init__('host_bridge')
        
        # UDP 소켓 초기화 (Local Loopback 127.0.0.1)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker 포즈 전송용 (Port 9091)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker 속도 수신용 (Port 9090)
        
        self.sock_recv.bind(('127.0.0.1', 9090))
        self.sock_recv.setblocking(False)

        # 1. RTAB-Map LIVO 50Hz 오도메트리 구독 (/rtabmap/odom)
        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        
        # 2. Go2 기본 라이다 포즈 백업 구독 (/utlidar/robot_pose)
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_callback, 10)
        
        # 3. 로봇 모터 명령 발행 토픽 (/cmd_vel) - Layer 1
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 4. Unitree Sport API 직접 발행 토픽 (/api/sport/request) - Layer 2 (Direct CycloneDDS)
        if HAS_UNITREE_API:
            self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
            self.get_logger().info("🐕 Unitree Sport API Direct Publisher (API ID 1008) 활성화 완료")
        else:
            self.sport_req_pub = None
            self.get_logger().warn("⚠️ unitree_api 모듈 미탑재 (표준 /cmd_vel 단독 발행 모드)")

        self.last_cmd_time = time.time()
        self.is_stopped = True

        # 50Hz (0.02초) 주기로 도커 속도 명령 수신 루프 실행
        self.create_timer(0.02, self.receive_loop)
        # 10Hz 워치독 타이머 (0.5초 무신호 시 자동 제동)
        self.create_timer(0.10, self.watchdog_loop)
        self.get_logger().info("🛡️ Host Bridge (Foxy) 가동 시작 (Dual-Layer: /cmd_vel + Sport API 1008, Magic 0x53324501)")

    def odom_callback(self, msg: Odometry):
        """RTAB-Map LIVO 50Hz Odometry -> Magic + 56B Double Packing -> Docker"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._send_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def pose_callback(self, msg: PoseStamped):
        """Go2 LiDAR Pose 백업 핸들러"""
        p = msg.pose.position
        q = msg.pose.orientation
        self._send_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _send_pose(self, x, y, z, qx, qy, qz, qw):
        raw_bytes = struct.pack('7d', x, y, z, qx, qy, qz, qw)
        crc = zlib.crc32(raw_bytes) & 0xFFFF
        # 패킷 규격: Magic(4B: 'I') + CRC(2B: 'H') + Pose(56B: '7d') = 62바이트
        packet = struct.pack('!IH', MAGIC_HEADER, crc) + raw_bytes
        try:
            self.sock_send.sendto(packet, ('127.0.0.1', 9091))
        except Exception:
            pass

    def _publish_actuation(self, vx: float, vy: float, vz: float, wx: float, wy: float, wz: float):
        """Dual-Layer Actuation: Standard ROS 2 /cmd_vel + Direct Sport API ID 1008"""
        # Safety speed limits
        vx = max(-MAX_VX, min(MAX_VX, vx))
        wz = max(-MAX_WZ, min(MAX_WZ, wz))

        # Layer 1: ROS 2 /cmd_vel
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        cmd.angular.x = float(wx)
        cmd.angular.y = float(wy)
        cmd.angular.z = float(wz)
        self.cmd_vel_pub.publish(cmd)

        # Layer 2: Unitree Official Sport API 1008
        if self.sport_req_pub is not None:
            try:
                sport_req = Request()
                sport_req.header.identity.api_id = 1008 # SportClient.Move
                sport_req.parameter = json.dumps({"x": float(vx), "y": float(vy), "z": float(wz)})
                self.sport_req_pub.publish(sport_req)
            except Exception:
                pass

        self.last_cmd_time = time.time()
        self.is_stopped = (abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4)

    def watchdog_loop(self):
        """0.5초 동안 Docker 명령 미수신 시 안전 자동 제동"""
        if not self.is_stopped and (time.time() - self.last_cmd_time > WATCHDOG_TIMEOUT_S):
            self.get_logger().warn("⚠️ [Watchdog Triggered] Docker 명령 지연 (>0.5s) -> 비상 정지 인가")
            self._publish_actuation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.is_stopped = True

    def receive_loop(self):
        """Docker로부터 속도 명령 수신 및 Magic/CRC 무결성 검증 후 Dual-Layer 발행"""
        try:
            data, addr = self.sock_recv.recvfrom(64)
            if len(data) == 54: # Magic(4B) + CRC(2B) + CmdVel(48B: 6d)
                magic, crc = struct.unpack('!IH', data[:6])
                payload = data[6:]
                if magic == MAGIC_HEADER and (zlib.crc32(payload) & 0xFFFF) == crc:
                    vx, vy, vz, wx, wy, wz = struct.unpack('6d', payload)
                    self._publish_actuation(vx, vy, vz, wx, wy, wz)
            elif len(data) == 48: # 레거시 48바이트 호환성 폴백
                vx, vy, vz, wx, wy, wz = struct.unpack('6d', data)
                self._publish_actuation(vx, vy, vz, wx, wy, wz)
        except BlockingIOError:
            pass
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HostBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
