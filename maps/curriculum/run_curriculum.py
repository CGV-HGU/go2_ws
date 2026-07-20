#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
S2E 4-Stage Curriculum Learning Sequential Pipeline Runner
Author: Lee Minseok
Description: 
    - 사용자(이민석) 독립 브랜치 전용 커리큘럼 러너 스크립트.
    - 건민 님의 코드베이스를 오염시키지 않으며, A6000/Pro 6000 서버에서 
      1단계~4단계 시나리오를 순차적으로 물려서 PPO 학습을 자동 자동화해주는 래퍼 도구.
"""

import os
import sys
import subprocess
import argparse
import shutil
from pathlib import Path

# 프로젝트 루트 및 설정 파일 경로 지정
REPO_ROOT = Path(__file__).resolve().parents[2]
URBAN_RL_ROOT = REPO_ROOT / "scratch" / "s2e-urban-rl"

# 4단계 커리큘럼 스테이지 YAML 경로 명세
CURRICULUM_STAGES = {
    1: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_stage1.yaml",
        "epochs": 100,
        "load_checkpoint": False,
        "description": "Stage 1: Clean Straight Sidewalk (Goal tracking prior)"
    },
    2: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_stage2.yaml",
        "epochs": 100,
        "load_checkpoint": True,
        "description": "Stage 2: Curved Sidewalk with Low Static Obstacles"
    },
    3: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_stage3.yaml",
        "epochs": 150,
        "load_checkpoint": True,
        "description": "Stage 3: Intersection with Blocked Sidewalk (Recovery drive)"
    },
    4: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_stage4.yaml",
        "epochs": 200,
        "load_checkpoint": True,
        "description": "Stage 4: Cluttered Sidewalk with Dynamic Pedestrians (Final Benchmark)"
    }
}

def run_command(cmd: list, cwd: Path) -> bool:
    print(f"\n[RUNNING CMD] {' '.join(cmd)}")
    try:
        # 실시간 로그 출력을 보면서 프로세스 서브런 실행
        result = subprocess.run(cmd, cwd=cwd, check=True)
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Command failed with exit code {e.returncode}")
        return False

def main():
    parser = argparse.ArgumentParser(description="S2E Curriculum Sequential Runner")
    parser.add_argument("--start_stage", type=int, default=1, choices=[1, 2, 3, 4], help="Stage to start training from")
    parser.add_argument("--end_stage", type=int, default=4, choices=[1, 2, 3, 4], help="Stage to end training at")
    parser.add_argument("--enable_cameras", action="store_true", default=True, help="Enable rendering cameras")
    parser.add_argument("--device", type=str, default="cuda:0", help="GPU device targeting (e.g. cuda:0)")
    args = parser.parse_args()

    print("==========================================================")
    print("      🚀 S2E 4-Stage Curriculum Learning Runner")
    print("==========================================================")
    print(f"Targeting s2e-urban-rl directory: {URBAN_RL_ROOT}")
    print(f"Executing Stages: {args.start_stage} ~ {args.end_stage}")
    print("==========================================================")

    # 이전 단계의 체크포인트 파일 트래킹 경로 (vLLM / rl-games checkpoint)
    # 실제 checkpoint 경로는 rl_games 훈련 스크립트 출력 경로에 매핑되어야 함
    last_checkpoint_path = ""

    for stage_id in range(args.start_stage, args.end_stage + 1):
        stage_cfg = CURRICULUM_STAGES[stage_id]
        print(f"\n[🚀 STARTING STAGE {stage_id}] {stage_cfg['description']}")
        
        # [★민석 자동 배포 패치] maps/curriculum/의 최신 YAML 설정을 s2e-urban-rl 공식 디렉토리로 동적 주입
        src_yaml = REPO_ROOT / "maps" / "curriculum" / f"go2_s2e_stage{stage_id}.yaml"
        dst_yaml = URBAN_RL_ROOT / stage_cfg["env_yaml"]
        
        if src_yaml.exists():
            print(f"[*] Copying latest curriculum config: {src_yaml.name} -> {dst_yaml}")
            dst_yaml.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_yaml, dst_yaml)
        else:
            print(f"[⚠️ WARNING] Source YAML {src_yaml} not found. Skipping auto-copy.")
            
        # 훈련 커맨드 빌드
        cmd = [
            "python", "urbansim/learning/RL/train.py",
            "--env", stage_cfg["env_yaml"],
            "--max_iterations", str(stage_cfg["epochs"])
        ]
        
        if args.enable_cameras:
            cmd.append("--enable_cameras")

        # 만약 2단계 이상이고 이전 체크포인트를 이어받는 경우 학습 가중치 연동 인자 강제 오버라이드
        # (주의: rl_games의 CLI 아규먼트 로드 규격에 맞춰 커맨드 빌드 필요)
        if stage_cfg["load_checkpoint"] and last_checkpoint_path:
            cmd.extend(["--checkpoint", last_checkpoint_path])
            print(f"[*] Inheriting weights from previous checkpoint: {last_checkpoint_path}")

        # 훈련 시작
        success = run_command(cmd, cwd=URBAN_RL_ROOT)
        if not success:
            print(f"\n[🛑 STOPPED] Training failed at Stage {stage_id}. Aborting pipeline.")
            sys.exit(1)

        # 훈련 성공 시 다음 단계를 위해 checkpoint 경로 업데이트
        # (rl_games 훈련 시 생성되는 latest checkpoint 매핑 예시)
        # 실제 checkpoint 이름 규칙 확인 필요 (예: go2_s2e_ram/checkpoint.pt)
        last_checkpoint_path = str(URBAN_RL_ROOT / "runs" / f"go2_s2e_stage{stage_id}" / "nn" / "latest.pt")
        print(f"[🟢 SUCCESS] Completed Stage {stage_id} successfully.")

    print("\n==========================================================")
    print("🎉 All Curriculum Stages Completed Successfully!")
    print("==========================================================")

if __name__ == "__main__":
    main()
