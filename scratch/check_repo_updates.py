import subprocess
import os

def check_updates():
    repos = {
        "s2e-nav-dataprocessing": "scratch/s2e-nav-dataprocessing",
        "s2e-urban-rl": "scratch/s2e-urban-rl"
    }
    
    print("="*60)
    print("       🔬 랩실 연동 저장소 원격 업데이트 점검 리포트")
    print("="*60)
    
    for name, path in repos.items():
        if os.path.exists(path):
            print(f"[*] {name} 상태 조회 중...")
            try:
                # 원격 변경사항 Fetch
                subprocess.run(
                    ["git", "fetch"], 
                    cwd=path, 
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL,
                    check=True
                )
                # 로컬 브랜치와 원격 브랜치 비교
                res = subprocess.run(
                    ["git", "status", "-uno"], 
                    cwd=path, 
                    capture_output=True, 
                    text=True, 
                    check=True
                )
                
                if "behind" in res.stdout:
                    print(f"  [⚠️ WARNING] 원격 저장소에 신규 업데이트(Commit)가 존재합니다!")
                    print(f"  --> 업데이트 적용하려면: cd {path} && git pull")
                elif "ahead" in res.stdout:
                    print(f"  [+] 로컬 변경사항이 원격보다 앞서 있습니다.")
                else:
                    print(f"  [🟢 OK] 최신 상태입니다. (Up-to-date)")
            except Exception as e:
                print(f"  [❌ ERROR] 상태 확인 실패: {e}")
        else:
            print(f"  [x] {name} 폴더가 {path}에 존재하지 않습니다.")
        print("-"*60)

if __name__ == "__main__":
    check_updates()
