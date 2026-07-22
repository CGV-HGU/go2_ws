import os
import shutil
import subprocess
import sys
from pathlib import Path

# 경로 정의
GO2_WS = Path(r"C:\Users\USER\Desktop\캡스톤\캡2-논문\go2_ws")
BUNDLE_DIR = GO2_WS / "scratch" / "optimized_bundle" / "urbansim_s2e_auto_optimized_bundle"
REPO_DIR = GO2_WS / "scratch" / "s2e-urban-rl"

def main():
    print("==========================================================")
    print("      🔧 Python-based Optimized Bundle Installer")
    print("==========================================================")
    print(f"Targeting Repository: {REPO_DIR}")
    
    # 1. 대상 디렉토리 보장
    (REPO_DIR / "urbansim" / "primitives" / "navigation").mkdir(parents=True, exist_ok=True)
    (REPO_DIR / "urbansim" / "envs" / "separate_envs").mkdir(parents=True, exist_ok=True)
    (REPO_DIR / "configs" / "env_configs" / "navigation").mkdir(parents=True, exist_ok=True)
    
    # 2. 파일 복사 수행
    copies = [
        ("overlay/urbansim/primitives/navigation/s2e_auto_mdp.py", "urbansim/primitives/navigation/s2e_auto_mdp.py"),
        ("overlay/urbansim/primitives/navigation/s2e_auto_env_cfg.py", "urbansim/primitives/navigation/s2e_auto_env_cfg.py"),
        ("overlay/urbansim/primitives/navigation/s2e_temporal_buffer.py", "urbansim/primitives/navigation/s2e_temporal_buffer.py"),
        ("overlay/urbansim/envs/separate_envs/s2e_auto_env.py", "urbansim/envs/separate_envs/s2e_auto_env.py"),
        ("overlay/configs/env_configs/navigation/coco_s2e_auto.yaml", "configs/env_configs/navigation/coco_s2e_auto.yaml"),
    ]
    
    for src_rel, dst_rel in copies:
        src_path = BUNDLE_DIR / src_rel
        dst_path = REPO_DIR / dst_rel
        print(f"[*] Copying {src_rel} -> {dst_rel}")
        shutil.copy2(src_path, dst_path)
        
    # 3. train.py와 play.py에 s2e_auto 연동을 위한 파이썬 패치 실행
    patch_script = BUNDLE_DIR / "tools" / "patch_train_play.py"
    print(f"[*] Running patch tool: python {patch_script} {REPO_DIR}")
    result = subprocess.run([sys.executable, str(patch_script), str(REPO_DIR)], capture_output=True, text=True)
    print(result.stdout)
    if result.stderr:
        print(f"[Warning/Error]: {result.stderr}")
        
    # 4. 바이트 컴파일 검사
    print("[*] Validating python byte-compilation...")
    compiles = [
        "urbansim/primitives/navigation/s2e_auto_mdp.py",
        "urbansim/primitives/navigation/s2e_auto_env_cfg.py",
        "urbansim/primitives/navigation/s2e_temporal_buffer.py",
        "urbansim/envs/separate_envs/s2e_auto_env.py",
        "urbansim/learning/RL/train.py",
        "urbansim/learning/RL/play.py",
    ]
    for rel_py in compiles:
        py_file = REPO_DIR / rel_py
        subprocess.run([sys.executable, "-m", "py_compile", str(py_file)], check=True)
        
    print("\n==========================================================")
    print("🟢 Optimized Bundle Installed Successfully!")
    print("==========================================================")

if __name__ == "__main__":
    main()
