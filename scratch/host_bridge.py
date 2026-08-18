import socket
import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist

class HostBridge(Node):
    def __init__(self):
        super().__init__('host_bridge')
        
        # UDP 소켓 초기화 (Local Loopback 127.0.0.1)
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker로 포즈 전송용 (Port 9091)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker로부터 속도 수신용 (Port 9090)
        
        # 9090 포트로 바인딩 (Docker S2E가 이 포트로 cmd_vel 명령을 보냄)
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
        self.get_logger().info("Host Bridge (Foxy) 가동 시작. UDP 송신: 9091, 수신 바인딩: 9090")

    def odom_callback(self, msg: Odometry):
        """RTAB-Map LIVO 50Hz Odometry -> 56바이트 바이너리 패킹 후 Docker 전송"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        data = struct.pack('7d', p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        try:
            self.sock_send.sendto(data, ('127.0.0.1', 9091))
        except Exception:
            pass

    def pose_callback(self, msg: PoseStamped):
        """Go2 LiDAR Pose 백업 핸들러"""
        p = msg.pose.position
        q = msg.pose.orientation
        data = struct.pack('7d', p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        try:
            self.sock_send.sendto(data, ('127.0.0.1', 9091))
        except Exception:
            pass

    def receive_loop(self):
        """Docker S2E로부터 6개 double 실수 (vx, vy, vz, wx, wy, wz = 48바이트) 수신 후 /cmd_vel 발행"""
        try:
            data, addr = self.sock_recv.recvfrom(48)
            if len(data) == 48:
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
