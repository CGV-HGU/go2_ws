"""Run the framework with the real ROS 2 Go2 Robot backend.

Set environment variables first:
    export QWEN_BASE_URL="https://your-qwen-endpoint/v1"
    export QWEN_API_KEY="..."
    export QWEN_MODEL="qwen3-vl-32b-thinking"

Then run:
    python examples/run_qwen_ros2.py --goal-x 5.0 --goal-y 0.0
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

# 패키지 절대경로 추가
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import rclpy
from nav_memory_qwen import NavMemoryAgent, NavAgentConfig, Ros2RobotBackend, OpenAICompatibleVLMClient


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, required=True, help="Goal X coordinate in map/odom frame")
    parser.add_argument("--goal-y", type=float, required=True, help="Goal Y coordinate in map/odom frame")
    parser.add_argument("--max-steps", type=int, default=20, help="Maximum navigation steps")
    parser.add_argument("--out", default="runs/qwen_ros2_episode", help="Output log directory")
    args = parser.parse_args()

    # 1. rclpy 초기화
    rclpy.init()

    # 2. 실물 ROS 2 로봇 백엔드 생성
    robot = Ros2RobotBackend()

    # 3. 비동기 rclpy.spin 처리를 위한 백그라운드 스레드 기동
    spin_thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spin_thread.start()

    try:
        # 4. OpenAI 호환 규격의 Qwen VL API 클라이언트 연동
        vlm = OpenAICompatibleVLMClient.from_env()
        
        # 5. 에이전트 인스턴스화
        agent = NavMemoryAgent(
            robot=robot,
            vlm_client=vlm,
            config=NavAgentConfig(
                max_steps=args.max_steps,
                log_full_vlm_input=False,
                force_front_view_waypoint=True # 안전 주행 모드
            ),
        )

        print(f"[*] Qwen 에피소딕 메모리 자율주행 기동 시작... 목표 좌표: ({args.goal_x}, {args.goal_y})")
        
        # 6. 목적지 도달 시까지 주행 시작
        result = agent.run_until_done(goal_map_xy=(args.goal_x, args.goal_y), max_steps=args.max_steps)
        
        out_dir = Path(args.out)
        agent.save_run(out_dir)
        
        print("\n" + "="*50)
        print("주행 완료 리포트:")
        print(result.summary())
        print(f"주행 로그 저장 완료: {out_dir.resolve()}")
        print("="*50)

    except KeyboardInterrupt:
        print("\n[!] 사용자 강제 종료 요청 수신.")
    finally:
        # 안전한 자원 해제
        robot.get_logger().info("ROS 2 백엔드 노드 종료 프로세스 시작.")
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
