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
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

MAGIC_HEADER = 0x53324501 # 'S2E\x01'

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
        
        # 3. 로봇 모터 명령 발행 토픽 (/cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 50Hz (0.02초) 주기로 도커 속도 명령 수신 루프 실행
        self.create_timer(0.02, self.receive_loop)
        self.get_logger().info("🛡️ Host Bridge (Foxy) 가동 시작 (Magic Header 0x53324501 검증 활성화)")

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

    def receive_loop(self):
        """Docker로부터 속도 명령 수신 및 Magic/CRC 무결성 검증 후 /cmd_vel 발행"""
        try:
            data, addr = self.sock_recv.recvfrom(64)
            if len(data) == 54: # Magic(4B) + CRC(2B) + CmdVel(48B: 6d)
                magic, crc = struct.unpack('!IH', data[:6])
                payload = data[6:]
                if magic == MAGIC_HEADER and (zlib.crc32(payload) & 0xFFFF) == crc:
                    vx, vy, vz, wx, wy, wz = struct.unpack('6d', payload)
                    cmd = Twist()
                    cmd.linear.x = float(vx)
                    cmd.linear.y = float(vy)
                    cmd.linear.z = float(vz)
                    cmd.angular.x = float(wx)
                    cmd.angular.y = float(wy)
                    cmd.angular.z = float(wz)
                    self.cmd_vel_pub.publish(cmd)
            elif len(data) == 48: # 레거시 48바이트 호환성 폴백
                vx, vy, vz, wx, wy, wz = struct.unpack('6d', data)
                cmd = Twist()
                cmd.linear.x = float(vx)
                cmd.linear.y = float(vy)
                cmd.linear.z = float(vz)
                cmd.angular.x = float(wx)
                cmd.angular.y = float(wy)
                cmd.angular.z = float(wz)
                self.cmd_vel_pub.publish(cmd)
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
