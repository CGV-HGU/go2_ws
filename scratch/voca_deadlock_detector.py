#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from s2e_vlm_msgs.msg import StampedPose

class VocaDeadlockDetector(Node):
    def __init__(self):
        super().__init__('voca_deadlock_detector')

        # ROS 2 파라미터 선언
        self.declare_parameter('check_duration', 3.0)        # 데드락 판정 시간 범위 (초)
        self.declare_parameter('min_command_speed', 0.15)    # 움직이려고 시도하는 기준 속도 (m/s)
        self.declare_parameter('stuck_distance_threshold', 0.15) # 움직이지 못한 판정 거리 (m)

        # 퍼블리셔 선언: VOCA/VLM 노드가 구독할 교착 상태 토픽 (/robot/status/deadlock)
        self.deadlock_pub = self.create_publisher(Bool, '/robot/status/deadlock', 10)

        # 서브스크라이버 선언
        self.create_subscription(Twist, '/s2e/controller/command', self.cmd_callback, 10)
        self.create_subscription(StampedPose, '/s2e/odometry/pose', self.odom_callback, 10)

        # 이력 관리 버퍼
        self.history = []  # [(timestamp, x, y, cmd_speed), ...]
        self.is_deadlocked = False

        # 10Hz (0.1초) 주기로 데드락 상태 체크 루프
        self.create_timer(0.1, self.check_deadlock_loop)
        self.get_logger().info("VOCA Deadlock Detector Node 가동 시작. 출력: /robot/status/deadlock")

        self.last_cmd = Twist()

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg

    def odom_callback(self, msg: StampedPose):
        # 현재 시간, 실측 위치, 지령 속도 계산
        now = time.time()
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 지령 속도의 크기 산출 (전진 속도 vx와 횡속도 vy 조합)
        cmd_speed = math.sqrt(self.last_cmd.linear.x**2 + self.last_cmd.linear.y**2)
        
        # 이력 추가
        self.history.append((now, x, y, cmd_speed))
        
        # 오래된 이력 버퍼 정제 (check_duration 초보다 오래된 데이터 제거)
        check_duration = self.get_parameter('check_duration').get_parameter_value().double_value
        cutoff = now - check_duration
        self.history = [h for h in self.history if h[0] >= cutoff]

    def check_deadlock_loop(self):
        check_duration = self.get_parameter('check_duration').get_parameter_value().double_value
        min_cmd_speed = self.get_parameter('min_command_speed').get_parameter_value().double_value
        stuck_dist_thresh = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value

        if len(self.history) < 5:
            return  # 충분한 데이터가 모일 때까지 스킵

        now = time.time()
        
        # 분석 대상 범위 필터링
        active_history = [h for h in self.history if h[0] >= (now - check_duration)]
        if len(active_history) < 2:
            return

        # 1. 제어 명령 조건 검사: 최근 3초 동안의 평균 지령 속도가 기준치 이상으로 달리고자 했는지 검사
        avg_cmd_speed = sum(h[3] for h in active_history) / len(active_history)
        
        # 2. 물리 변위 조건 검사: 최근 3초 동안 실제로 움직인 직선 거리 계산
        start_x, start_y = active_history[0][1], active_history[0][2]
        end_x, end_y = active_history[-1][1], active_history[-1][2]
        actual_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)

        # 3. 데드락 판정 조건 결합
        # (이동하라는 명령 속도는 높은데, 실제 변위는 미미할 경우 교착으로 인식)
        deadlock_detected = (avg_cmd_speed >= min_cmd_speed) and (actual_distance < stuck_dist_thresh)

        # 상태 변화가 있을 때만 토픽 발행 및 로깅
        if deadlock_detected != self.is_deadlocked:
            self.is_deadlocked = deadlock_detected
            msg = Bool()
            msg.data = self.is_deadlocked
            self.deadlock_pub.publish(msg)
            
            if self.is_deadlocked:
                self.get_logger().warn(
                    f"[🚨 DEADLOCK DETECTED] 명령 속도: {avg_cmd_speed:.3f} m/s | "
                    f"최근 {check_duration}초 실측 변위: {actual_distance:.3f} m (기준치: {stuck_dist_thresh}m 미만)"
                )
            else:
                self.get_logger().info("[🟢 DEADLOCK RESOLVED] 로봇이 다시 원활하게 주행하기 시작했습니다.")
        else:
            # 매 스텝마다 상태 유지 발행 (생존 신고/상태 고정)
            msg = Bool()
            msg.data = self.is_deadlocked
            self.deadlock_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VocaDeadlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
