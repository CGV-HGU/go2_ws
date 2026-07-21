import os
import sys
import subprocess
import argparse
import shutil
from pathlib import Path

# 프로젝트 루트 및 설정 파일 경로 지정
REPO_ROOT = Path(__file__).resolve().parents[2]
URBAN_RL_ROOT = REPO_ROOT / "scratch" / "s2e-urban-rl"

# 건민 님이 공식 반영한 4단계 커리큘럼 스테이지 YAML 경로 명세 (S2E V2 표준 규격)
CURRICULUM_STAGES = {
    0: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_auto_stage0.yaml",
        "epochs": 100,
        "load_checkpoint": False,
        "description": "Stage 0: Straight Walkway (Goal 6-10m, clean/sparse objects)"
    },
    1: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_auto_stage1.yaml",
        "epochs": 100,
        "load_checkpoint": True,
        "description": "Stage 1: Straight Walkway (Goal 8-14m, sparse objects)"
    },
    2: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_auto_stage2.yaml",
        "epochs": 120,
        "load_checkpoint": True,
        "description": "Stage 2: Curved Walkway (Goal 10-16m, bottleneck/slalom objects)"
    },
    3: {
        "env_yaml": "configs/env_configs/navigation/go2_s2e_auto_stage3.yaml",
        "epochs": 150,
        "load_checkpoint": True,
        "description": "Stage 3: Complex Clutter Walkway (Goal 10-20m, dense/slalom/curve-clutter)"
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
    parser = argparse.ArgumentParser(description="S2E Official Curriculum Sequential Runner")
    parser.add_argument("--start_stage", type=int, default=0, choices=[0, 1, 2, 3], help="Stage to start training from")
    parser.add_argument("--end_stage", type=int, default=3, choices=[0, 1, 2, 3], help="Stage to end training at")
    parser.add_argument("--enable_cameras", action="store_true", default=True, help="Enable rendering cameras")
    parser.add_argument("--device", type=str, default="cuda:0", help="GPU device targeting (e.g. cuda:0)")
    args = parser.parse_args()

    print("==========================================================")
    print("      🚀 S2E V2 Official Curriculum Learning Runner")
    print("==========================================================")
    print(f"Targeting s2e-urban-rl directory: {URBAN_RL_ROOT}")
    print(f"Executing Stages: Stage {args.start_stage} ~ Stage {args.end_stage}")
    print("==========================================================")

    # 이전 단계의 체크포인트 파일 트래킹 경로
    last_checkpoint_path = ""

    for stage_id in range(args.start_stage, args.end_stage + 1):
        stage_cfg = CURRICULUM_STAGES[stage_id]
        print(f"\n[🚀 STARTING STAGE {stage_id}] {stage_cfg['description']}")
        
        # 훈련 커맨드 빌드
        cmd = [
            "python", "urbansim/learning/RL/train.py",
            "--env", stage_cfg["env_yaml"],
            "--max_iterations", str(stage_cfg["epochs"])
        ]
        
        if args.enable_cameras:
            cmd.append("--enable_cameras")

        # 만약 이전 체크포인트를 이어받는 경우 학습 가중치 연동 인자 강제 오버라이드
        if stage_cfg["load_checkpoint"] and last_checkpoint_path:
            cmd.extend(["--checkpoint", last_checkpoint_path])
            print(f"[*] Inheriting weights from previous stage checkpoint: {last_checkpoint_path}")

        # 훈련 시작
        success = run_command(cmd, cwd=URBAN_RL_ROOT)
        if not success:
            print(f"\n[🛑 STOPPED] Training failed at Stage {stage_id}. Aborting pipeline.")
            sys.exit(1)

        # 훈련 성공 시 생성된 latest.pth 가중치를 특정 스테이지용으로 백업 및 트래킹 업데이트
        # rl_games의 go2_s2e_ram 실험 기본 가중치 경로
        origin_checkpoint = URBAN_RL_ROOT / "runs" / "go2_s2e_ram" / "nn" / "go2_s2e_ram.pth"
        backup_checkpoint = URBAN_RL_ROOT / "runs" / "go2_s2e_ram" / "nn" / f"go2_s2e_auto_stage{stage_id}_final.pth"
        
        if origin_checkpoint.exists():
            print(f"[*] Backing up stage {stage_id} checkpoint: {origin_checkpoint.name} -> {backup_checkpoint.name}")
            shutil.copy2(origin_checkpoint, backup_checkpoint)
            last_checkpoint_path = str(backup_checkpoint)
        else:
            # 훈련 체크포인트 백업 실패 시 디버깅 처리
            print(f"[⚠️ WARNING] Checkpoint {origin_checkpoint} not found. Searching for alternative...")
            nn_dir = URBAN_RL_ROOT / "runs" / "go2_s2e_ram" / "nn"
            if nn_dir.exists():
                pth_files = list(nn_dir.glob("*.pth"))
                if pth_files:
                    # 가장 최근에 수정된 .pth 파일을 찾아서 백업
                    latest_file = max(pth_files, key=lambda p: p.stat().st_mtime)
                    print(f"[*] Found alternative latest checkpoint: {latest_file.name}. Copying to backup...")
                    shutil.copy2(latest_file, backup_checkpoint)
                    last_checkpoint_path = str(backup_checkpoint)
                else:
                    print("[🛑 ERROR] No .pth checkpoint files found. Cannot continue to next stage.")
                    sys.exit(1)
            else:
                print("[🛑 ERROR] runs/go2_s2e_ram/nn directory does not exist. Cannot continue.")
                sys.exit(1)

        print(f"[🟢 SUCCESS] Completed Stage {stage_id} successfully.")

    print("\n==========================================================")
    print("🎉 All Curriculum Stages Completed Successfully!")
    print("==========================================================")

if __name__ == "__main__":
    main()
