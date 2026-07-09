import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from s2e_vlm_msgs.msg import StampedPose

class DockerBridge(Node):
    def __init__(self):
        super().__init__('docker_bridge')
        
        # UDP 소켓 초기화
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Host로 속도 명령 전송용
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Host로부터 포즈 수신용
        
        # 9091 포트로 바인딩 (Host가 이 포트로 포즈를 보냄)
        self.sock_recv.bind(('127.0.0.1', 9091))
        self.sock_recv.setblocking(False)

        # ROS 2 토픽 구독 및 발행
        self.create_subscription(Twist, '/s2e/controller/command', self.cmd_callback, 10)
        self.pose_pub = self.create_publisher(StampedPose, '/s2e/odometry/pose', 10)

        # 50Hz (0.02초) 주기로 수신 루프 실행
        self.create_timer(0.02, self.receive_loop)
        self.get_logger().info("Docker Bridge (Jazzy) 가동 시작. UDP 송신: 9090, 수신 바인딩: 9091")

    def cmd_callback(self, msg: Twist):
        # vx, vy, vz, wx, wy, wz (6개 double 실수 = 48바이트) 바이너리 압축
        data = struct.pack('6d', 
                           msg.linear.x, msg.linear.y, msg.linear.z,
                           msg.angular.x, msg.angular.y, msg.angular.z)
        try:
            # Host의 9090 포트로 전송
            self.sock_send.sendto(data, ('127.0.0.1', 9090))
        except Exception:
            pass

    def receive_loop(self):
        try:
            # 7개 double 실수 (x, y, z, qx, qy, qz, qw = 56바이트) 수신
            data, addr = self.sock_recv.recvfrom(56)
            if len(data) == 56:
                x, y, z, qx, qy, qz, qw = struct.unpack('7d', data)
                
                # 프레임워크 규격인 StampedPose로 가공
                pose_msg = StampedPose()
                now_msg = self.get_clock().now().to_msg()
                pose_msg.header.stamp = now_msg
                pose_msg.header.frame_id = 'odom'
                pose_msg.child_frame_id = 'base_link'
                pose_msg.source_stamp = now_msg
                pose_msg.processed_stamp = now_msg
                
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                
                pose_msg.confidence = 1.0
                pose_msg.status = 'ACTIVE'
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
