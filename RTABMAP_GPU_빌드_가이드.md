# 🚀 RTAB-Map CUDA (GPU 가속) 빌드 및 적용 가이드

본 문서는 **Unitree Go2** 및 **Intel RealSense D435i** 환경에서 RTAB-Map SLAM 구동 시 발생하는 노드 업데이트 지연(Frame Drop) 현상을 해결하기 위해, **OpenCV CUDA 및 RTAB-Map GPU 가속 환경을 소스 컴파일로 구축하는 상세 가이드**입니다.

---

## 📌 1. 개요 및 배경

### 1.1 노드 지연 원인 및 GPU 가속의 필요성
- **기존 문제**: ROS 기본 패키지(`ros-foxy-rtabmap-ros`)는 CPU 전용으로 빌드되어 있어, RealSense D435i의 High-Hz 이미지와 Depth PointCloud 처리 시 CPU 병목으로 인해 맵 업데이트가 느려지거나 프레임 드롭이 발생합니다.
- **파라미터 튜닝의 한계**: 특징점 수(`MaxFeatures`)를 줄이면 속도는 향상되나, Loop Closure 감지율 및 SLAM 맵 정밀도가 하락합니다.
- **최대 전원 모드의 한계**: `jetson_clocks`로 CPU 클럭을 고정하면 발열 및 로봇 배터리 소모가 급증합니다.

### 1.2 RTAB-Map GPU 가속 원리 (Issue #561 기준)
- RTAB-Map 자체는 단일 GPU 토글 옵션을 제공하지 않으며, 의존성 라이브러리인 **OpenCV CUDA 기능**을 연동하여 GPU 가속을 수행합니다.
- **핵심 가속 포인트**:
  - **특징점 추출기 (Feature Detector)**: `ORB-CUDA` (`Kp/DetectorStrategy=6`, `ORB/Gpu=true`), `FAST-CUDA` 등
  - **와트당 성능(Perf/Watt) 향상**: CPU 100% 풀가동 대비 GPU 병렬 처리로 1000개 이상의 특징점을 수 ms 만에 처리하여 배터리 효율과 실시간성(30fps)을 동시에 확보합니다.

---

## 📋 2. 사전 준비 사항 (Prerequisites)

Jetson 로봇 PC(Ubuntu 20.04 + ROS 2 Foxy) 접속 후 아래 항목을 확인합니다.

```bash
# 1. CUDA 및 nvcc 설치 확인
nvcc --version

# 2. 필수 빌드 도구 및 의존성 패키지 설치
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    libpcl-dev libsqlite3-dev liboctomap-dev
```

---

## 🛠️ 3. 단계별 GPU 소스 빌드 절차

### STEP 1: 기존 ROS 바이너리 RTAB-Map 제거
공식 패키지와 소스 빌드 버전 간 라이브러리 충돌을 방지합니다.
```bash
sudo apt remove -y ros-foxy-rtabmap ros-foxy-rtabmap-ros
```

---

### STEP 2: CUDA 지원 OpenCV 소스 컴파일 및 설치

> **주의**: Jetson Orin/Xavier 기종에 맞는 `CUDA_ARCH_BIN` 아키텍처 버전을 지정해야 합니다. (Orin: `8.7`, Xavier: `7.2`, TX2: `6.2`)

```bash
# 1. 작업 디렉토리 생성
cd /home/unitree
mkdir -p opencv_build && cd opencv_build

# 2. OpenCV 및 opencv_contrib 소스 다운로드 (동일 버전 4.5.4 이상 권장)
git clone -b 4.5.4 https://github.com/opencv/opencv.git
git clone -b 4.5.4 https://github.com/opencv/opencv_contrib.git

# 3. 빌드 디렉토리 이동
cd opencv
mkdir build && cd build

# 4. CMake 설정 (CUDA 옵션 포함)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D ENABLE_FAST_MATH=1 \
      -D CUDA_FAST_MATH=1 \
      -D CUDA_ARCH_BIN=8.7 \
      -D WITH_GSTREAMER=ON \
      -D WITH_LIBV4L=ON \
      -D BUILD_EXAMPLES=OFF ..

# 5. 컴파일 및 설치 (스레드 수에 맞춰 -j 옵션 조정)
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### 설치 확인:
```bash
python3 -c "import cv2; print(cv2.__version__); print('CUDA Devices:', cv2.cuda.getCudaEnabledDeviceCount())"
# 출력 결과: CUDA Devices: 1 이어야 정상입니다.
```

---

### STEP 3: RTAB-Map C++ Standalone 라이브러리 빌드

```bash
cd /home/unitree
git clone https://github.com/introlab/rtabmap.git
cd rtabmap/build

# CMake 실행 시 OpenCV CUDA 연동 여부 출력 로그를 반드시 확인합니다.
cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

> **로그 확인 포인트**:
> `With OpenCV CUDA = YES` 및 `With ORB_OCTREE = YES` 문구가 표시되어야 합니다.

```bash
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

### STEP 4: rtabmap_ros (ROS 2 Foxy) 워크스페이스 빌드

```bash
cd /home/unitree/go2_ws/src
git clone -b foxy-devel https://github.com/introlab/rtabmap_ros.git

cd /home/unitree/go2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 🏃‍♂️ 4. GPU 파라미터 적용 및 검증 방법

### 4.1 Launch 파일 파라미터 업데이트 ([run_map.sh](file:///C:/Users/hmkan/go2_ws/run_map.sh))
RTAB-Map 실행 시 `ORB-CUDA` 및 `FAST-CUDA` 추출기를 사용하도록 launch 인자를 추가합니다:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start --Kp/DetectorStrategy 6 --ORB/Gpu true --FAST/Gpu true --Kp/MaxFeatures 1000 --Vis/MinInliers 5 --Grid/MaxObstacleHeight 1.5 --Grid/CellSize 0.1 --Grid/RayTracing true --Optimizer/GravitySigma 0.3' \
    frame_id:=camera_link \
    visual_odometry:=true \
    odom_topic:=/odom \
    ...
```

### 4.2 GPU 모니터링 및 성능 검증
로봇 PC에서 터미널을 열어 GPU 점유율과 토픽 발행 주파수를 확인합니다:

```bash
# 1. GPU 및 시스템 자원 점유율 모니터링
tegrastats

# 2. RTAB-Map 맵 토픽 발행 Hz 확인 (정상 시 15~30Hz 유지)
ros2 topic hz /rtabmap/map
```

---

## ❓ 5. 자주 묻는 질문 (Troubleshooting)

1. **`cv_bridge`와 소스 빌드 OpenCV 충돌 문제**:
   - ROS 2 Foxy의 `cv_bridge`가 시스템 기본 OpenCV 4.2.0을 참조하여 오류가 발생하는 경우, `cv_bridge` 패키지도 소스 빌드한 OpenCV 4.5.4를 바라보도록 워크스페이스 내에서 재빌드합니다.
2. **CUDA Out of Memory (OOM) 발생 시**:
   - `Kp/MaxFeatures`를 1000개 수준으로 제한하여 GPU 메모리 버퍼를 유지합니다.
