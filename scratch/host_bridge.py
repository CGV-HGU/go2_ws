import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class HostBridge(Node):
    def __init__(self):
        super().__init__('host_bridge')
        
        # UDP 소켓 초기화
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker로 포즈 전송용
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Docker로부터 속도 수신용
        
        # 9090 포트로 바인딩 (Docker가 이 포트로 명령을 보냄)
        self.sock_recv.bind(('127.0.0.1', 9090))
        self.sock_recv.setblocking(False)

        # ROS 2 토픽 구독 및 발행
        self.create_subscription(PoseStamped, '/utlidar/robot_pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 50Hz (0.02초) 주기로 수신 루프 실행
        self.create_timer(0.02, self.receive_loop)
        self.get_logger().info("Host Bridge (Foxy) 가동 시작. UDP 송신: 9091, 수신 바인딩: 9090")

    def pose_callback(self, msg: PoseStamped):
        # x, y, z, qx, qy, qz, qw (7개 double 실수 = 56바이트) 바이너리 압축
        data = struct.pack('7d', 
                           msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                           msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        try:
            # Docker 컨테이너의 9091 포트로 전송
            self.sock_send.sendto(data, ('127.0.0.1', 9091))
        except Exception:
            pass

    def receive_loop(self):
        try:
            # 6개 double 실수 (vx, vy, vz, wx, wy, wz = 48바이트) 수신
            data, addr = self.sock_recv.recvfrom(48)
            if len(data) == 48:
                vx, vy, vz, wx, wy, wz = struct.unpack('6d', data)
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.linear.z = vz
                cmd.angular.x = wx
                cmd.angular.y = wy
                cmd.angular.z = wz
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
