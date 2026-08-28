# ⚡ [COOKBOOK] Unitree Go2 ESCAPE-Nav 초고속 에러 해결 쿡북 (Quick Lookup & Remedy)

> **문서 목적**: 실물 로봇 현장에서 에러 발생 시, 터미널에 출력된 에러 로그 문자열로 즉시 검색하여 **10초 안에 원인과 1줄 복구 명령어를 찾아 실행**하기 위한 랩실 영구 기술 자산 색인집입니다.

---

## 🔍 [터미널 에러 문자열별 10초 해결 색인 (Index)]

| 터미널 에러 문자열 (Grep Keyword) | 해당 계층 | 핵심 원인 | 1줄 즉각 해결 명령어 |
| :--- | :---: | :--- | :--- |
| **`Did not receive data since 5 seconds!`** | RTAB-Map | 센서 토픽 중 1개 이상 0Hz | `./map_headless.sh` |
| **`bind udp port failed`** | 라이다 드라이버 | 이전 라이다 프로세스 6201 포트 점유 | `fuser -k 6201/udp` (필요한 경우 터미널에서 명시적 `sudo`) |
| **`ImportError: libopencv_hdf.so.4.5`** | 전면 카메라 | glibc 런타임 링커 경로 누락 | `sudo ldconfig /home/unitree/opencv_build/opencv/build/lib` |
| **`0 Hz` on `/utlidar/cloud`** | 4D 라이다 | 메인보드 원시 점군 Mute 상태 | `smart app toggle` or `unitree_ros2 service request` |
| **`Could not find a connection between odom and base_link`** | TF2 | 로봇 와상/정지 상태 TF 미발행 | `go2_native_sensor_node.py` 50Hz 타이머 TF 활성화 |
| **`RELIABILITY_QOS_POLICY incompatible`** | ROS 2 CLI | Best-Effort vs Reliable 불일치 | `ros2 topic hz <topic> --qos-reliability reliable` |
| **`404 Model Not Found`** on vLLM | VLM 서버 | 서버에 올라간 모델명과 클라이언트 불일치 | `vlm_client.py`의 Auto-Discovery (`/v1/models`) 사용 |
| **`UDP 9090 connection refused`** | Docker 브릿지 | 도커 네트워크 모드 `bridge` 고립 | `docker run --net=host` (`network_mode: host`) 적용 |
| **`Messages of type 0 arrived out of order`** | 카메라 스트림 | 카메라 이중 오픈 충돌 | `pkill -9 -f go2_front_camera` 후 단일 노드 기동 |

---

## 🛠️ [상세 복구 처방전 (Detailed Remedies)]

### 1. `Did not receive data since 5 seconds!` (RTAB-Map 블로킹)
* **원인**: RTAB-Map `approx_sync`는 구독 중인 모든 토픽(`/camera/front/image_raw`, `/camera/front/camera_info`, `/utlidar/cloud`)에 패킷이 와야 프레임을 생성합니다.
* **진단**:
  ```bash
  ros2 topic hz /camera/front/image_raw /utlidar/cloud /odom
  ```
* **해결**:
  ```bash
  # 0Hz인 토픽 확인 후 브링업 재실행
  ./map_headless.sh
  ```

---

### 2. `bind udp port failed` (라이다 포트 충돌)
* **원인**: 이전 실행 인스턴스가 비정상 종료되어 UDP 6201 포트를 여전히 점유하고 있음.
* **해결**:
  ```bash
  fuser -k 6201/udp
  sleep 0.5
  bash scratch/start_unitree_lidar.sh
  ```

---

### 3. `ImportError: libopencv_hdf.so.4.5` (OpenCV 동적 라이브러리 누락)
* **원인**: 시스템 기본 라이브러리 경로에 Jetson 빌드 OpenCV 라이브러리가 미등록됨.
* **해결**:
  ```bash
  echo "/home/unitree/opencv_build/opencv/build/lib" | sudo tee /etc/ld.so.conf.d/opencv.conf
  sudo ldconfig
  ```

---

### 4. VLM 서버 모델 교체 시 불일치 에러 (`404 Model Not Found`)
* **원인**: 서버 관리자가 Qwen3.8-27B에서 Qwen3-VL-32B 등으로 교체했을 때 클라이언트가 과거 모델명을 요청함.
* **해결**:
  ```bash
  # 자동 감지 검증 스크립트 실행 (100% 무설정 자동 감지)
  python3 scratch/test_docker_to_sport_cmd_vel_pipeline.py
  ```

---

### 5. 도커에서 속도 명령 보냈는데 로봇이 안 움직임 (Watchdog 또는 포트 고립)
* **원인**: 도커 컨테이너가 `network_mode: host`로 실행되지 않아 `127.0.0.1:9090`이 호스트 OS에 도달하지 않음.
* **진단 및 해결**:
  ```bash
  # 1. 54-Byte UDP 루프백 스트레스 테스트 실행
  python3 scratch/test_docker_50hz_stress.py
  # 2. 호스트 브릿지 수신 확인
  ros2 topic echo /cmd_vel
  ```
