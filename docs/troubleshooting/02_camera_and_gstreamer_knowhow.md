# 📷 [Know-How 02] Go2 전면 초광각 카메라 & GStreamer 영상 스트리밍 정밀 해설서

> **대상 시스템**: Go2 전면 초광각 카메라, RTP 멀티캐스트 `230.1.1.1:1720`, OpenCV GStreamer, ROS 2 이미지 퍼블리셔  
> **문서 목적**: 전면 영상 H.264 수신, OpenCV 동적 링커 경로, 멀티캐스트 라우팅, 워커 스레드 30fps 동기화 전수 해설

---

## 1. 🔍 RTP 멀티캐스트 수신 인터페이스 설정

1. **카메라 영상 스트림 규격**:
   - 해상도: $1280 \times 720$ (또는 $1920 \times 1080$), 30 fps
   - 프로토콜: RTP / H.264 멀티캐스트 주소 `230.1.1.1:1720`
2. **GStreamer 수신 인터페이스 고정**:
   - publisher의 `udpsrc`가 `multicast-iface=eth0`를 지정하므로 별도 root route는 필요하지 않습니다. Go2 direct path만 확인합니다:
   ```bash
   ip -4 route get 192.168.123.161
   ```

---

## 2. ⚡ OpenCV `dlopen` 라이브러리 경로 누락 해결법

* **증상**: Python 실행 시 `ImportError: libopencv_hdf.so.4.5: cannot open shared object file` 발생.
* **영구 해결책**:
  ```bash
  echo "/home/unitree/opencv_build/opencv/build/lib" | sudo tee /etc/ld.so.conf.d/opencv.conf
  sudo ldconfig
  ```

---

## 3. 🧵 GStreamer 중복 오픈 충돌 방지 및 30fps 워커 스레드 동기화

* 단일 소켓에 복수 프로세스가 접근하면 영상 프레임이 손실됩니다.
* 전용 백그라운드 캡처 스레드를 두고 `threading.Lock`을 통해 ROS 2 콜백과 디커플링하여 30fps 고정 스트리밍을 보장합니다.
