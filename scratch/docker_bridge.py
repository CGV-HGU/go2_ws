#!/usr/bin/env python3
"""
Docker Bridge (ROS 2 Jazzy / Python 3.12 in Ubuntu 24.04 Container)
========================================================================================
- Receives 62-byte pose packet (Magic 4B + CRC 2B + 56B Pose) from Host (127.0.0.1:9091).
- Validates Magic Header (0x53324501) & CRC -> Publishes to /s2e/odometry/pose (StampedPose).
- Subscribes to /s2e/controller/command (Twist) -> Packs with Magic + CRC (54B) -> Sends to Host (127.0.0.1:9090).
"""

import socket
import struct
import zlib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from s2e_vlm_msgs.msg import StampedPose

MAGIC_HEADER = 0x53324501 # 'S2E\x01'

class DockerBridge(Node):
    def __init__(self):
        super().__init__('docker_bridge')
        
        # UDP 소켓 초기화
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Host로 속도 명령 전송용 (Port 9090)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Host로부터 포즈 수신용 (Port 9091)
        
        self.sock_recv.bind(('127.0.0.1', 9091))
        self.sock_recv.setblocking(False)

        # ROS 2 토픽 구독 및 발행
        self.create_subscription(Twist, '/s2e/controller/command', self.cmd_callback, 10)
        self.pose_pub = self.create_publisher(StampedPose, '/s2e/odometry/pose', 10)

        # 50Hz (0.02초) 주기로 수신 루프 실행
        self.create_timer(0.02, self.receive_loop)
        self.get_logger().info("🛡️ Docker Bridge (Jazzy) 가동 시작 (Magic Header 0x53324501 검증 활성화)")

    def cmd_callback(self, msg: Twist):
        """속도 명령 -> Magic + CRC + 48B Double Packing -> Host (Port 9090)"""
        raw_bytes = struct.pack('6d', 
                                msg.linear.x, msg.linear.y, msg.linear.z,
                                msg.angular.x, msg.angular.y, msg.angular.z)
        crc = zlib.crc32(raw_bytes) & 0xFFFF
        packet = struct.pack('!IH', MAGIC_HEADER, crc) + raw_bytes
        try:
            self.sock_send.sendto(packet, ('127.0.0.1', 9090))
        except Exception:
            pass

    def receive_loop(self):
        """Host로부터 포즈 수신 및 Magic/CRC 무결성 검증 후 /s2e/odometry/pose 발행"""
        try:
            data, addr = self.sock_recv.recvfrom(128)
            payload = None
            if len(data) == 62: # Magic(4B) + CRC(2B) + Pose(56B: 7d)
                magic, crc = struct.unpack('!IH', data[:6])
                raw = data[6:]
                if magic == MAGIC_HEADER and (zlib.crc32(raw) & 0xFFFF) == crc:
                    payload = raw
            elif len(data) == 56: # 레거시 56바이트 호환성 폴백
                payload = data

            if payload and len(payload) == 56:
                x, y, z, qx, qy, qz, qw = struct.unpack('7d', payload)
                
                pose_msg = StampedPose()
                now_msg = self.get_clock().now().to_msg()
                pose_msg.header.stamp = now_msg
                pose_msg.header.frame_id = 'odom'
                pose_msg.child_frame_id = 'base_link'
                pose_msg.source_stamp = now_msg
                pose_msg.processed_stamp = now_msg
                
                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)
                
                self.pose_pub.publish(pose_msg)
        except BlockingIOError:
            pass
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = DockerBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
