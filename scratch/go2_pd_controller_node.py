#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32MultiArray
from s2e_vlm_msgs.msg import StampedPose

class Go2PDControllerNode(Node):
    def __init__(self):
        super().__init__('go2_pd_controller_node')

        # ROS 2 파라미터 선언 (런타임에서 동적 변경 가능)
        self.declare_parameter('waypoint_dt', 0.4)          # S2E V2의 dt=2.0s 와 매핑하기 위해 0.4s 로 기본값 수정 (5 * 0.4 = 2.0)
        self.declare_parameter('lookahead_idx', 4)         # 몇 번째 웨이포인트를 목표로 바라볼지 (4 = 5번째 점, 2.0초 뒤)
        self.declare_parameter('kx', 1.0)                  # X축 (선속도) 제어 게인
        self.declare_parameter('ky', 1.0)                  # Y축 (횡속도) 제어 게인
        self.declare_parameter('k_heading', 1.0)           # 회전 각속도 제어 게인
        self.declare_parameter('max_vx', 1.0)              # 최대 전진 속도 (m/s)
        self.declare_parameter('max_vy', 0.6)              # 최대 횡 이동 속도 (m/s)
        self.declare_parameter('max_wz', 0.8)              # 최대 회전 속도 (rad/s)
        self.declare_parameter('min_walk_speed', 0.05)     # 실물 Go2의 최저 기동 속도 Deadband (0.05 m/s)

        # 퍼블리셔 선언: DockerBridge로 속도 전달 (/s2e/controller/command)
        self.cmd_pub = self.create_publisher(Twist, '/s2e/controller/command', 10)

        # 서브스크라이버 선언: S2E 모델이 예측하는 10개 웨이포인트 구독
        # 다양한 인터페이스에 대응하기 위해 PoseArray와 Float32MultiArray 두 형태를 모두 지원하도록 콜백 작성
        self.create_subscription(PoseArray, '/s2e/policy/waypoints_pose', self.waypoints_pose_callback, 10)
        self.create_subscription(Float32MultiArray, '/s2e/policy/waypoints_array', self.waypoints_array_callback, 10)
        
        # 오도메트리 토픽 구독 (모니터링 및 디버깅용)
        self.create_subscription(StampedPose, '/s2e/odometry/pose', self.odom_callback, 10)

        self.last_pose = None
        self.get_logger().info("Go2 PD Controller Node 가동 시작. 입력: /s2e/policy/waypoints_*, 출력: /s2e/controller/command")

    def odom_callback(self, msg: StampedPose):
        self.last_pose = msg

    def waypoints_pose_callback(self, msg: PoseArray):
        """PoseArray 타입으로 들어오는 10개 웨이포인트 처리 콜백"""
        waypoints = []
        for pose in msg.poses:
            # S2E 모델은 2D 평면 주행이므로 x, y 정보만 추출
            waypoints.append([pose.position.x, pose.position.y])
        self.compute_and_publish_cmd(waypoints)

    def waypoints_array_callback(self, msg: Float32MultiArray):
        """Float32MultiArray 타입 (10x2 = 20차원 flat list)으로 들어오는 웨이포인트 처리 콜백"""
        if len(msg.data) < 2:
            return
        
        # flat list를 [N, 2] 구조로 리스트 변환
        waypoints = []
        for i in range(0, len(msg.data), 2):
            if i + 1 < len(msg.data):
                waypoints.append([msg.data[i], msg.data[i+1]])
        self.compute_and_publish_cmd(waypoints)

    def compute_and_publish_cmd(self, waypoints):
        """궤적 제어기 수식을 적용하여 속도 명령 계산 및 발행"""
        if not waypoints:
            self.get_logger().warn("수신된 웨이포인트 데이터가 비어 있습니다.")
            return

        # 런타임 파라미터 값 읽기
        waypoint_dt = self.get_parameter('waypoint_dt').get_parameter_value().double_value
        lookahead_idx = self.get_parameter('lookahead_idx').get_parameter_value().integer_value
        kx = self.get_parameter('kx').get_parameter_value().double_value
        ky = self.get_parameter('ky').get_parameter_value().double_value
        k_heading = self.get_parameter('k_heading').get_parameter_value().double_value
        max_vx = self.get_parameter('max_vx').get_parameter_value().double_value
        max_vy = self.get_parameter('max_vy').get_parameter_value().double_value
        max_wz = self.get_parameter('max_wz').get_parameter_value().double_value
        min_walk_speed = self.get_parameter('min_walk_speed').get_parameter_value().double_value

        # lookahead 인덱스 바운더리 보호
        idx = min(max(int(lookahead_idx), 0), len(waypoints) - 1)
        lookahead_time = (idx + 1) * waypoint_dt

        # 타겟 웨이포인트 지정 (Body-frame 상대 변위)
        target_x, target_y = waypoints[idx]

        # 1. 선속도 명령 계산 (kx, ky 이득 반영)
        vx = kx * target_x / lookahead_time
        vy = ky * target_y / lookahead_time

        # 2. 각속도 명령 계산 (atan2를 통해 바라봐야 할 타겟 각도 오차 산출 후 k_heading 반영)
        heading_error = math.atan2(target_y, target_x)
        wz = k_heading * heading_error / lookahead_time

        # 3. 실물 로봇개 최저 기동 속도 보정 (Deadband Clamping)
        # 속도가 계산되었으나 최저 속도(min_walk_speed)보다 낮으면 로봇이 굳어버리므로 최저 속도로 밀어 올림.
        # 단, 아주 정지 상태에 근접한 속도(0.01 m/s 미만)는 안전을 위해 완전 정지(0.0)로 보냄.
        speed_magnitude = math.sqrt(vx**2 + vy**2)
        if 0.01 < speed_magnitude < min_walk_speed:
            scale_factor = min_walk_speed / speed_magnitude
            vx *= scale_factor
            vy *= scale_factor
        elif speed_magnitude <= 0.01:
            vx = 0.0
            vy = 0.0

        # 4. 물리적 안전 속도 제한 클램핑
        vx = max(min(vx, max_vx), -max_vx)
        vy = max(min(vy, max_vy), -max_vy)
        wz = max(min(wz, max_wz), -max_wz)

        # 5. Twist 메시지 포장 및 발행
        cmd_msg = Twist()
        cmd_msg.linear.x = float(vx)
        cmd_msg.linear.y = float(vy)
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = float(wz)

        self.cmd_pub.publish(cmd_msg)

        # 디버깅 로그 출력
        self.get_logger().debug(
            f"TargetIdx: {idx} | Target: ({target_x:.3f}, {target_y:.3f}) | "
            f"Cmd: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Go2PDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
