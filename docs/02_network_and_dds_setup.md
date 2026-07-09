# ❓ 네트워크 & DDS 환경 구성 (Network & DDS Setup)

본 문서는 실하드웨어(Jetson Orin NX)와 외부 GPU 연산 서버를 학교 무선 네트워크 및 VPN망으로 연결할 때 발생하는 CycloneDDS 통신 장애 요인을 분석하고, 이를 우회하기 위한 DDS 및 루프백 소켓 브릿지 셋업 가이드를 기술함.

---

## 📌 1. 네트워크 및 DDS 장애 시나리오 (DDS Issues)

### 1.1 CycloneDDS 인터페이스 바인딩 오류 (로컬 통신 두절)
*   **문제**: 로봇 onboard PC가 학교망(Wi-Fi 등)에 연결되거나 Netbird VPN 인터페이스(`netbird0`)가 생성되는 순간, CycloneDDS가 디폴트 바인딩 인터페이스를 외부망으로 임의 전환함. 이로 인해 로봇 내부 제어기(IP `192.168.123.161`)와의 기가비트 이더넷 로컬 DDS 통신이 즉시 두절됨.
*   **대책**: `cyclonedds.xml` 파일 내 `<NetworkInterfaceAddress>` 태그에 로봇 내부 LAN 포트인 `eth0`를 하드코딩 고정 바인딩해야 함.

### 1.2 학교망/VPN의 멀티캐스트(Multicast) 차단 (원격 노드 검색 실패)
*   **문제**: ROS 2는 기본적으로 UDP 멀티캐스트 방식으로 노드를 탐색함. 그러나 공공 학교망이나 WireGuard 기반 Netbird VPN 터널은 멀티캐스트 통신을 기본 차단함. 따라서 물리 로봇 측 노드와 연산을 담당할 외부 서버 측 노드가 서로를 전혀 인지하지 못하는 차단 상황이 연출됨.
*   **대책**: 멀티캐스트를 차단하고 양단 PC의 고정 IP를 `cyclonedds.xml`의 유니캐스트 피어 주소 리스트(`<Peers>`)로 사전에 명기하여 다이렉트 매핑 구조로 전환함.

### 1.3 대용량 센서 데이터 전송 시 Jetson CPU 과부하 (제어 주기 지연)
*   **문제**: 4D LiDAR 포인트 클라우드나 고해상도 카메라 원본 프레임을 Netbird 암호화 채널을 거쳐 외부 PC로 전송할 때, 암호화 연산(WireGuard 패킷 인코딩)으로 인해 Jetson Orin NX의 CPU 점유율이 100%를 초과하여 핵심 보행 제어 스레드가 지연되고 로봇이 다운될 수 있음.
*   **대책**: 원격 VPN 통로로는 모델이 요구하는 가벼운 제어 정보(10x2 Trajectory 및 Odom 데이터)만 송수신하고, 무거운 센서 데이터의 외부 스트리밍을 원천 격리하여 바이패스하는 대역폭 필터링 설계가 요구됨.

---

## 🏃 2. 화요일 연동 검증 시나리오 및 우회 전략 (Tuesday Test Plan)

화요일 실제 로봇(Foxy, 20.04) 접속 시, 도커 컨테이너(Jazzy, 24.04) 환경과의 통신 호환성을 검증하는 1순위 시나리오 및 통신 두절 시 우회 방안.

### 2.1 [1순위] CycloneDDS 루프백 다이렉트 검증 (가장 단순함)
*   **환경 설정 (호스트 및 도커 양측 선언)**:
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0
    ```
    *(도커 컨테이너 구동 시 `--net=host` 옵션 필수 포함)*
*   **테스트 1 (Foxy ➔ Jazzy)**:
    *   호스트(Foxy) 터미널:
        ```bash
        ros2 topic pub /test_odom nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}}" -r 10
        ```
    *   도커(Jazzy) 터미널:
        ```bash
        ros2 topic echo /test_odom
        ```
        *(DDS 패킷이 도커 격리망을 뚫고 Jazzy 내에서 역직렬화가 정상 작동하는지 모니터링)*
*   **테스트 2 (Jazzy ➔ Foxy)**:
    *   도커(Jazzy) 터미널:
        ```bash
        ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10
        ```
    *   호스트(Foxy) 터미널:
        ```bash
        ros2 topic echo /cmd_vel
        ```
        *(Jazzy에서 보낸 속도 제어 메시지가 Foxy 호스트의 드라이버 단에 무손실 도달하는지 확인)*

### 2.2 [2순위] 통신 실패 시 우회 전략 (대안)
위 다이렉트 통신이 미들웨어 버전 불일치로 실패할 경우 시도할 우회로:
*   **Zenoh Bridge 연동**: 호스트와 도커 양측에 `zenoh-bridge-dds`를 켜서 버전 독립적인 고속 통신 터널 개설.
*   **파이썬 소켓 브릿지**: 양측에 ROS 2 통신망 영향이 없는 일반 Python 소켓 스크립트를 띄워 로우 바이트 데이터를 강제 전송 및 바이패스.
