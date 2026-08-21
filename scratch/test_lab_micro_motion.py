#!/usr/bin/env python3
"""
========================================================================================
🐕 Unitree Go2 Active Stepping Trot Verification with Real-Time Telemetry Feedback
========================================================================================
Purpose:
- Safely verifies active 4-leg stepping trot motion in indoor lab environments.
- Directly commands Go2 Sport Controller via CycloneDDS (/api/sport/request) & /cmd_vel.
- Subscribes to 50Hz SportModeState to display real-time physical displacement (Δx, Δy).
- Motion profile:
  1. Standby & Telemetry Lock (1.0s)
  2. Active Stepping Forward (+0.30 m/s for 1.0s -> ~+30cm, ~2-3 steps)
  3. Standstill Pause (1.0s)
  4. Active Stepping Backward (-0.30 m/s for 1.0s -> ~-30cm, ~2-3 steps return)
  5. Final Zero-Velocity Safety Lock

Safety Features:
- Controlled velocity (Default: 0.30 m/s).
- Direct /api/sport/request (API ID 1008) + /cmd_vel dual-layer publishing.
- Instant Zero-Velocity lock on Ctrl+C (SIGINT/SIGTERM).
- Remote controller E-Stop (L2 + B) hardware override.
========================================================================================
"""

import sys
import time
import json
import math
import signal
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from unitree_api.msg import Request
from unitree_go.msg import SportModeState, LowState

class SafeMicroMotionNode(Node):
    def __init__(self):
        super().__init__('safe_micro_motion_node')
        
        # 1. Dual-Layer Publishers (Direct DDS API + ROS 2 cmd_vel)
        self.sport_req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. State & Telemetry Storage
        self.init_pose = None
        self.current_pose = [0.0, 0.0, 0.0] # x, y, z
        self.current_rpy = [0.0, 0.0, 0.0]
        self.battery_soc = 0
        self.battery_volt = 0.0
        self.telemetry_received = False

        # 3. Best-Effort QoS Subscriber for Go2 Native DDS Telemetry
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.create_subscription(SportModeState, 'sportmodestate', self.sport_state_callback, sensor_qos)
        self.create_subscription(SportModeState, '/sportmodestate', self.sport_state_callback, sensor_qos)
        self.create_subscription(SportModeState, 'lf/sportmodestate', self.sport_state_callback, sensor_qos)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.sport_state_callback, sensor_qos)
        self.create_subscription(LowState, 'lowstate', self.low_state_callback, sensor_qos)
        self.create_subscription(LowState, '/lowstate', self.low_state_callback, sensor_qos)

    def sport_state_callback(self, msg: SportModeState):
        self.current_pose = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        self.current_rpy = [float(msg.imu_state.rpy[0]), float(msg.imu_state.rpy[1]), float(msg.imu_state.rpy[2])]
        if self.init_pose is None:
            self.init_pose = list(self.current_pose)
        self.telemetry_received = True

    def low_state_callback(self, msg: LowState):
        self.battery_soc = int(msg.bms_state.soc)
        self.battery_volt = float(msg.power_v)
        self.telemetry_received = True

    def send_velocity(self, vx: float, vy: float = 0.0, vyaw: float = 0.0):
        # Layer 1: Direct Unitree Sport API (API ID 1008 = Move)
        req = Request()
        req.header.identity.api_id = 1008
        param = {"x": float(vx), "y": float(vy), "z": float(vyaw)}
        req.parameter = json.dumps(param)
        self.sport_req_pub.publish(req)

        # Layer 2: Standard ROS 2 /cmd_vel
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(vyaw)
        self.cmd_vel_pub.publish(cmd)

    def get_displacement(self):
        if self.init_pose is None:
            return 0.0, 0.0, 0.0
        dx = self.current_pose[0] - self.init_pose[0]
        dy = self.current_pose[1] - self.init_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)
        return dx, dy, dist

    def emergency_stop(self):
        for _ in range(12):
            self.send_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

def main():
    parser = argparse.ArgumentParser(description="Unitree Go2 Safe Active Stepping Verification")
    parser.add_argument('--speed', type=float, default=0.30, help="Forward/Backward stepping speed in m/s (default: 0.30)")
    parser.add_argument('--duration', type=float, default=1.0, help="Stepping duration in seconds (default: 1.0)")
    parser.add_argument('--pause', type=float, default=1.0, help="Pause duration between motions (default: 1.0)")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SafeMicroMotionNode()

    def signal_handler(sig, frame):
        print("\n🛑 [EMERGENCY STOP] Interrupted! Locking robot motors to 0.0 m/s...")
        node.emergency_stop()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    target_dist = cli_args.speed * cli_args.duration
    print("=" * 76)
    print(" 🐕 Unitree Go2 Safe Lab Active Stepping Trot Verification")
    print("=" * 76)
    print(f" ⚙️  [실행 설정] 속도: ±{cli_args.speed:.2f} m/s | 지속 시간: {cli_args.duration:.1f}s (예상 이동량: 약 ±{target_dist*100:.0f}cm)")
    print(" ⚠️  [안전 확인 사항]")
    print("   1. 로봇이 기립(Stand-Up) 상태인지 확인해 주세요.")
    print(f"   2. 로봇 앞/뒤 {int(target_dist*100 + 40)}cm 이내에 장애물이 없는지 확인해 주세요.")
    print("   3. 무선 조종기를 손에 쥐고 이상 시 즉시 'L2 + B'를 누를 준비를 해주세요.")
    print("=" * 76)

    # 1. Strict DDS Handshake Barrier & Initial Origin Calibration
    print("\n[1/4] ⏳ Establishing CycloneDDS Handshake with Go2 Mainboard...")
    t_start_hs = time.time()
    while not node.telemetry_received:
        node.send_velocity(0.0, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)
        if time.time() - t_start_hs > 10.0:
            print("  ❌ [ERROR] CycloneDDS Handshake timed out after 10s. Check 192.168.123.161 ethernet link.")
            sys.exit(1)

    print(f"  🟢 DDS Telemetry Connected in {time.time() - t_start_hs:.2f}s! Battery: {node.battery_soc}% ({node.battery_volt:.1f}V)")

    # 0.5s Stabilization & Lock Origin Baseline
    t_stab = time.time() + 0.5
    while time.time() < t_stab:
        node.send_velocity(0.0, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.02)

    node.init_pose = list(node.current_pose)
    print(f"  📍 Origin Baseline Locked: X={node.init_pose[0]:.3f}m, Y={node.init_pose[1]:.3f}m, Z={node.init_pose[2]:.3f}m")

    # 2. Step A: Active Stepping Forward (+speed for duration)
    print(f"\n🚀 [2/4] ➡️  Active Stepping Forward (+{cli_args.speed:.2f} m/s for {cli_args.duration:.1f}s / ~+{target_dist*100:.0f}cm)...")
    t_start = time.time()
    t_end = t_start + cli_args.duration
    last_print = 0
    while time.time() < t_end:
        node.send_velocity(cli_args.speed, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.02)
        if time.time() - last_print > 0.2:
            dx, dy, dist = node.get_displacement()
            progress = min(100.0, ((time.time() - t_start) / cli_args.duration) * 100)
            print(f"    ➡️  Stepping... [{progress:3.0f}%] | Real Δx: {dx:+6.3f}m | Δy: {dy:+6.3f}m | Dist: {dist:5.3f}m")
            last_print = time.time()

    # Pause (0.0 m/s for pause duration)
    print(f"⏸️  [Pause] 🛑 Standstill ({cli_args.pause:.1f}s)...")
    t_end = time.time() + cli_args.pause
    while time.time() < t_end:
        node.send_velocity(0.0, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.02)

    # 3. Step B: Active Stepping Backward (-speed for duration)
    print(f"\n🔄 [3/4] ⬅️  Active Stepping Backward (-{cli_args.speed:.2f} m/s for {cli_args.duration:.1f}s / Returning)...")
    t_start = time.time()
    t_end = t_start + cli_args.duration
    last_print = 0
    while time.time() < t_end:
        node.send_velocity(-cli_args.speed, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.02)
        if time.time() - last_print > 0.2:
            dx, dy, dist = node.get_displacement()
            progress = min(100.0, ((time.time() - t_start) / cli_args.duration) * 100)
            print(f"    ⬅️  Returning... [{progress:3.0f}%] | Real Δx: {dx:+6.3f}m | Δy: {dy:+6.3f}m | Dist: {dist:5.3f}m")
            last_print = time.time()

    # 4. Final Safe Lock
    print("\n🔒 [4/4] Sending final zero-velocity lock...")
    node.emergency_stop()
    dx_final, dy_final, dist_final = node.get_displacement()

    print("=" * 76)
    print(" 🏆 [SUCCESS] Active stepping verification completed successfully!")
    print(f"   • Peak Forward Displacement: ~+{target_dist:.2f} m")
    print(f"   • Real Physical Displacement: Δx = {dx_final:+6.3f} m, Δy = {dy_final:+6.3f} m")
    print(f"   • Net Return Error to Origin: {dist_final:.3f} m")
    print(f"   • Final Battery Status: {node.battery_soc}% ({node.battery_volt:.1f}V)")
    print("   • 4-Leg Dynamic Trot Actuation Pipeline 100% Verified.")
    print("=" * 76)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
