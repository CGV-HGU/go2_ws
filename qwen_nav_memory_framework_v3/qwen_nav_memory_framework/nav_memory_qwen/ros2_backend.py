import os
import time
import math
import tempfile
from typing import Tuple, Sequence, List, Optional
from pathlib import Path

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Twist
from s2e_vlm_msgs.msg import StampedPose, Trajectory2D
from s2e_vlm_msgs.action import Rotate

from PIL import Image as PILImage
import numpy as np

from .schema import Observation, ObservationView, RelativePose2D, RobotState, ActionOutcome, normalize_angle_rad

class Ros2RobotBackend(Node):
    """Qwen VLA 에이전트와 실하드웨어 ROS 2 통신망을 다이렉트로 연결하는 실제 배포용 백엔드"""

    def __init__(self, node_name: str = "qwen_ros2_backend"):
        super().__init__(node_name)
        
        # ROS 2 구독 설정
        self.create_subscription(StampedPose, "/s2e/odometry/pose", self._pose_callback, 10)
        self.create_subscription(Image, "/s2e/sensors/camera/image", self._image_callback, 10)
        
        # ROS 2 발행 설정
        self.trajectory_pub = self.create_publisher(Trajectory2D, "/s2e/e2e/trajectory", 10)
        
        # Rotate Action Client 설정
        self.rotate_client = ActionClient(self, Rotate, "/s2e/controller/rotate")
        
        # 로봇 상태값 저장 변수
        self.current_pose: StampedPose | None = None
        self.current_image: Image | None = None
        self.sequence_id = f"seq_{int(time.time())}"
        self.frame_index = 0
        
        # 임시 이미지 폴더 생성
        self.temp_dir = Path(tempfile.gettempdir()) / "qwen_images"
        self.temp_dir.mkdir(parents=True, exist_ok=True)
        
        # 카메라 투영 매개변수 (RealSense D435i 기준 기본값)
        self.camera_fov_h = math.radians(69.0) # 수평 화각
        self.camera_height = 0.25 # Go2 로봇 몸체 기준 카메라 높이 (m)
        self.camera_tilt = math.radians(-15.0) # 전방 바라보는 각도 (바닥 투영용)

        self.get_logger().info("Ros2RobotBackend (Qwen 실물 드라이버) 초기화 완료.")

    def _pose_callback(self, msg: StampedPose):
        self.current_pose = msg

    def _image_callback(self, msg: Image):
        self.current_image = msg

    def get_robot_state(self) -> RobotState:
        """현재 LIO 기반 오도메트리 위치를 획득"""
        if self.current_pose is None:
            return RobotState(map_xy=(0.0, 0.0), heading_rad=0.0)
        
        # 쿼터니언에서 Yaw 변환
        qz = self.current_pose.pose.orientation.z
        qw = self.current_pose.pose.orientation.w
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        return RobotState(map_xy=(x, y), heading_rad=yaw)

    def _save_raw_image_to_jpeg(self) -> str:
        """ROS 2 이미지 메시지를 PIL을 거쳐 임시 JPEG 파일로 안전 변환 저장"""
        if self.current_image is None:
            # 실하드웨어에 이미지가 아직 안 돌면 더미 이미지 생성 공급
            dummy_img = PILImage.new("RGB", (640, 480), (128, 128, 128))
            file_path = self.temp_dir / f"dummy_{self.frame_index}.jpg"
            dummy_img.save(file_path)
            return str(file_path)

        # CvBridge 종속성 충돌을 차단하기 위한 raw 바이트 수동 변환식
        w, h = self.current_image.width, self.current_image.height
        if self.current_image.encoding in {"rgb8", "bgr8"}:
            img_data = np.frombuffer(self.current_image.data, dtype=np.uint8).reshape((h, w, 3))
            if self.current_image.encoding == "bgr8":
                img_data = img_data[:, :, ::-1] # BGR -> RGB 변환
            pil_img = PILImage.fromarray(img_data)
        else:
            # 기타 단색/그레이스케일 대응
            pil_img = PILImage.new("RGB", (w, h), (128, 128, 128))

        file_path = self.temp_dir / f"frame_{self.frame_index}_{int(time.time())}.jpg"
        pil_img.save(file_path)
        return str(file_path)

    def get_observation(self) -> Observation:
        """VLM 추론용 현재 카메라 입력 데이터프레임 구성"""
        self.frame_index += 1
        ts = int(time.time() * 1000)
        img_path = self._save_raw_image_to_jpeg()
        
        w = self.current_image.width if self.current_image else 640
        h = self.current_image.height if self.current_image else 480
        
        # 전면 뷰를 담은 ObservationView 반환
        views = [ObservationView(view_id=0, view_type="front", relative_heading_deg=0.0, image=img_path, timestamp_ms=ts)]
        return Observation(
            mode="current_only",
            sequence_id=self.sequence_id,
            frame_index=self.frame_index,
            image_width=w,
            image_height=h,
            views=views,
            timestamp_ms=ts
        )

    def rotate(self, yaw_deg: float) -> ActionOutcome:
        """Rotate Action Server를 호출하여 실물 로봇 제자리 회전 제어"""
        self.get_logger().info(f"제자리 회전 기동 시작: {yaw_deg:.2f}도")
        
        if not self.rotate_client.wait_for_server(timeout_sec=5.0):
            return ActionOutcome(action="rotate", success=False, message="Rotate Action Server 응답 없음")

        goal = Rotate.Goal()
        goal.target_yaw_delta_deg = float(yaw_deg)
        goal.max_yaw_rate_deg_s = 30.0
        goal.tolerance_deg = 3.0
        goal.timeout_s = 10.0

        send_goal_future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return ActionOutcome(action="rotate", success=False, message="회전 요청이 거절됨")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        outcome = ActionOutcome(
            action="rotate",
            success=result.success,
            rotated_deg=result.final_yaw_delta_deg,
            odom_delta=RelativePose2D(dx_m=0.0, dy_m=0.0, dyaw_deg=result.final_yaw_delta_deg),
            message=result.message
        )
        return outcome

    def execute_waypoint(self, *, view_type: str, view_id: int, point_px: Tuple[int, int], ttl_ms: int) -> ActionOutcome:
        """VLA가 예측한 이미지 픽셀 좌표를 로컬 3D 공간으로 변환하여 실물 궤적 주행 명령 하달"""
        px_x, px_y = point_px
        w = self.current_image.width if self.current_image else 640
        h = self.current_image.height if self.current_image else 480
        
        # 1. 픽셀 좌표 -> 수평 편차 각도 (Horizontal Angle)
        # 이미지 좌우 편차를 구함 (센터 = 0)
        norm_x = (px_x - (w / 2.0)) / (w / 2.0)
        yaw_angle = -norm_x * (self.camera_fov_h / 2.0)
        
        # 2. 바닥 평면 투영 공식을 사용해 거리(Distance) 계산
        # 픽셀 세로 축을 기준으로 바닥 평면과의 교점을 계산해 약식 전방 거리를 구함
        norm_y = ((h / 2.0) - px_y) / (h / 2.0)
        pitch_angle = self.camera_tilt + (norm_y * math.radians(42.0 / 2.0)) # 수직 화각 약 42도 가정
        
        if pitch_angle >= 0:
            # 수평선 이상을 찍었을 때 예외 방지용 최소 기본 전진 거리 부여
            distance = 0.8
        else:
            distance = self.camera_height / math.tan(-pitch_angle)
            distance = max(0.2, min(2.0, distance)) # 제어 안전 거리 클리핑

        # 3. 로컬 3D Target 좌표 생성 (base_link 기준)
        dx = distance * math.cos(yaw_angle)
        dy = distance * math.sin(yaw_angle)

        self.get_logger().info(f"픽셀 {point_px} 투영 결과: dx={dx:.2f}m, dy={dy:.2f}m")

        # 4. Trajectory2D 메시지 생성 및 발행
        traj = Trajectory2D()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = "base_link"
        
        # 5개 점으로 이어지는 단순 직선 경로 생성
        for step in range(1, 6):
            pt = Point32()
            pt.x = float(dx * (step / 5.0))
            pt.y = float(dy * (step / 5.0))
            pt.z = 0.0
            traj.points.append(pt)
            
        traj.goal_point_base_link.x = float(dx)
        traj.goal_point_base_link.y = float(dy)
        
        # 궤적 송출 시작
        self.trajectory_pub.publish(traj)
        
        # LIO 포즈를 관찰하며 대상 목표점 근방(< 0.15m)에 수렴하는지 모니터링 시작
        start_state = self.get_robot_state()
        start_x, start_y = start_state.map_xy
        start_time = time.time()
        success = False
        
        while time.time() - start_time < (ttl_ms / 1000.0):
            # rclpy 이벤트 대기 처리
            rclpy.spin_once(self, timeout_sec=0.05)
            
            curr_state = self.get_robot_state()
            cx, cy = curr_state.map_xy
            
            # 이동한 절대 거리 계산
            dist_moved = math.hypot(cx - start_x, cy - start_y)
            # 타겟에 대한 잔여 거리 오차
            remaining = math.hypot(dx - dist_moved, dy) # 단순 전진 위주 약식 계산
            
            if remaining < 0.15:
                success = True
                break
                
            time.sleep(0.05)

        end_state = self.get_robot_state()
        ex, ey = end_state.map_xy
        moved_dist = math.hypot(ex - start_x, ey - start_y)
        
        return ActionOutcome(
            action="go",
            success=success,
            moved_distance_m=moved_dist,
            odom_delta=RelativePose2D(dx_m=moved_dist, dy_m=0.0, dyaw_deg=math.degrees(end_state.heading_rad - start_state.heading_rad)),
            message=f"moved {moved_dist:.2f}m"
        )

    def capture_views(self, yaw_offsets_deg: Sequence[float], mode: str = "directed_sweep") -> Observation:
        """Directed Sweep 구현 - 로봇을 회전시키며 해당 뷰의 이미지를 임시 취득"""
        ts = int(time.time() * 1000)
        views: List[ObservationView] = []
        
        # 1. 원본 자세 기록
        orig_state = self.get_robot_state()
        
        # 2. 각 오프셋 각도로 제자리 회전을 돌아 사진 캡쳐 후 원복
        for i, offset in enumerate(yaw_offsets_deg):
            self.rotate(offset)
            time.sleep(0.3) # 카메라 포커스 및 잔여 진동 진정 대기
            
            img_path = self._save_raw_image_to_jpeg()
            views.append(ObservationView(
                view_id=i + 1,
                view_type="directed",
                relative_heading_deg=float(offset),
                image=img_path,
                timestamp_ms=ts
            ))
            # 다시 제자리로 복귀
            self.rotate(-offset)
            
        self.frame_index += 1
        return Observation(
            mode=mode,
            sequence_id=self.sequence_id,
            frame_index=self.frame_index,
            image_width=640,
            image_height=480,
            views=views,
            timestamp_ms=ts
        )
