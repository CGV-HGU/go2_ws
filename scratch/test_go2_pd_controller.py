#!/usr/bin/env python3
import math

class Go2TrajectoryControllerSimulator:
    """go2_pd_controller_node.py의 제어 로직만 분리하여 시뮬레이션 및 검증하는 테스트 클래스"""
    def __init__(self, waypoint_dt=0.2, lookahead_idx=4, kx=1.0, ky=1.0, k_heading=1.0,
                 max_vx=1.0, max_vy=0.6, max_wz=0.8):
        self.waypoint_dt = waypoint_dt
        self.lookahead_idx = lookahead_idx
        self.kx = kx
        self.ky = ky
        self.k_heading = k_heading
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz

    def compute_cmd(self, target_x, target_y):
        # lookahead 시간 계산 (index 4 기준 1.0초)
        lookahead_time = (self.lookahead_idx + 1) * self.waypoint_dt
        
        # 1. 선속도 명령 계산
        vx = self.kx * target_x / lookahead_time
        vy = self.ky * target_y / lookahead_time

        # 2. 각속도 명령 계산 (atan2 안전 방어)
        if abs(target_x) < 1e-6 and abs(target_y) < 1e-6:
            heading_error = 0.0
        else:
            heading_error = math.atan2(target_y, target_x)
            
        wz = self.k_heading * heading_error / lookahead_time

        # 3. 물리 제한 클램핑
        vx_clamped = max(min(vx, self.max_vx), -self.max_vx)
        vy_clamped = max(min(vy, self.max_vy), -self.max_vy)
        wz_clamped = max(min(wz, self.max_wz), -self.max_wz)

        return {
            "raw": (vx, vy, wz),
            "clamped": (vx_clamped, vy_clamped, wz_clamped)
        }

def run_diagnostic():
    sim = Go2TrajectoryControllerSimulator()
    
    # 대표적인 테스트용 2D 웨이포인트 세트 정의 (상대 좌표 x, y)
    test_cases = [
        # (테스트 설명, target_x, target_y)
        ("1. 직진 주행 (먼 거리)", 2.0, 0.0),
        ("2. 직진 주행 (가까운 거리)", 0.5, 0.0),
        ("3. 전방 좌측 대각선 주행", 1.0, 1.0),
        ("4. 전방 우측 대각선 주행", 1.0, -1.0),
        ("5. 좌측 직각 게걸음 (횡이동)", 0.0, 2.0),
        ("6. 우측 직각 게걸음 (횡이동)", 0.0, -2.0),
        ("7. 완전 후진 주행", -1.0, 0.0),
        ("8. 제자리 정지 상태 (Zero Input)", 0.0, 0.0),
        ("9. 극단적인 물리 한계 초과점 (발산 방지 테스트)", 10.0, 10.0),
    ]

    print("="*95)
    print(f"{'테스트 시나리오':<30} | {'입력 Target (X, Y)':<18} | {'계산 속도 [vx, vy, wz]':<38}")
    print("="*95)

    success = True
    for desc, tx, ty in test_cases:
        res = sim.compute_cmd(tx, ty)
        raw_vx, raw_vy, raw_wz = res["raw"]
        vx, vy, wz = res["clamped"]
        
        print(f"{desc:<30} | ({tx:>5.2f}, {ty:>5.2f})       | vx={vx:>6.3f}, vy={vy:>6.3f}, wz={wz:>6.3f} (제한 전: {raw_vx:>5.2f})")

        # 물리 법칙 자가 테스트 및 단언(Assertions)
        if "직진" in desc:
            if abs(vy) > 1e-6 or abs(wz) > 1e-6:
                print(f"[❌ FAIL] 직진 주행인데 vy={vy} 또는 wz={wz} 값이 존재함!")
                success = False
        elif "좌측" in desc:
            if vy < 0 or wz < 0:
                print(f"[❌ FAIL] 좌회전/좌횡이동인데 속도 방향이 음수임 (vy={vy}, wz={wz})")
                success = False
        elif "우측" in desc:
            if vy > 0 or wz > 0:
                print(f"[❌ FAIL] 우회전/우횡이동인데 속도 방향이 양수임 (vy={vy}, wz={wz})")
                success = False
        elif "정지" in desc:
            if abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6:
                print(f"[❌ FAIL] 정지 상태인데 속도 명령이 나감 (vx={vx}, vy={vy}, wz={wz})")
                success = False

        # 속도 클램핑 검증
        if abs(vx) > sim.max_vx + 1e-6 or abs(vy) > sim.max_vy + 1e-6 or abs(wz) > sim.max_wz + 1e-6:
            print(f"[❌ FAIL] 최대 속도 한계 제약을 초과하는 속도가 산출됨! (vx={vx}, vy={vy}, wz={wz})")
            success = False

    print("="*95)
    if success:
        print("[🟢 SUCCESS] 모든 Kinematics 물리 검증 테스트를 통과했습니다! 제어 수식이 완벽히 강건합니다.")
    else:
        print("[❌ FAIL] 일부 제어 케이스에서 비정상적인 거동이 감지되었습니다. 로그를 확인하세요.")

if __name__ == "__main__":
    run_diagnostic()
