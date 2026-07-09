import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2.common.channel import ChannelFactoryInitialize
from unitree_sdk2.go2.sport.sport_client import SportClient

class PythonDirectDriver(Node):
    def __init__(self):
        super().__init__('python_direct_driver')
        
        # DDS 바인딩할 네트워크 인터페이스 지정 (기본값: eth0)
        interface = "eth0"
        if len(sys.argv) > 1 and not sys.argv[1].startswith('-'):
            interface = sys.argv[1]
            
        self.get_logger().info(f"Unitree SDK2 파이썬 채널 초기화 인터페이스: {interface}")
        try:
            # DDS 채널 공장 초기화
            ChannelFactoryInitialize(0, interface)
            self.client = SportClient()
            self.client.SetTimeout(10.0)
            self.client.Init()
            self.get_logger().info("Unitree SDK2 SportClient 포트 연동 완료.")
        except Exception as e:
            self.get_logger().error(f"Unitree SDK2 초기화 실패: {e}")
            sys.exit(1)

        # /cmd_vel 속도 명령 구독 설정
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.get_logger().info("호스트단 /cmd_vel 토픽 구독 대기 중. Go2 구동 준비 완료.")

    def cmd_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vyaw = msg.angular.z
        try:
            # 로봇 스포츠 API로 선속도/각속도 직접 전달
            self.client.Move(vx, vy, vyaw)
        except Exception as e:
            self.get_logger().warn(f"로봇 Move 명령 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PythonDirectDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.get_logger().info("종료 신호 수신. 로봇 보호를 위해 안전 제동(Damp) 명령 인가.")
            node.client.Damp() # 안전 착지 제어
        except Exception:
            pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
