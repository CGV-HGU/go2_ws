# 📡 로봇(Go2) - Jetson - 외부망(노트북) 간 ROS 2 DDS 통신 설정 가이드

Unitree Go2와 같은 4족 보행 로봇 환경에서는 **로봇 본체 내부망(192.168.123.xx)**과 **외부 개발망(학교 유선랜/Wi-Fi)**이라는 2개의 독립된 네트워크가 동시에 가동됩니다. 

이 문서에서는 이 다중 네트워크 환경에서 **로봇 ➔ Jetson ➔ 외부 노트북** 간에 끊김 없고 막힘없는 ROS 2 DDS 통신을 구축하기 위한 최적의 네트워크 라우팅 및 DDS 바인딩 설정을 설명합니다.

---

## 1. 근본적인 네트워크 통신 장애 원인

1. **멀티-NIC(네트워크 카드) 우선순위 문제 (Multi-NIC Routing)**:
   * Jetson 보드는 로봇 내부망(`eth0`)과 외부망(`wlan0` 또는 USB 랜카드 `eth1`)에 동시에 연결되어 있습니다.
   * OS에 외부망 게이트웨이가 잡히면, CycloneDDS는 기본 네트워크 카드만 사용하려고 하므로 로봇 본체(`192.168.123.161`)와의 통신이 마비됩니다.
2. **공공 학교망의 멀티캐스트(Multicast) 차단**:
   * ROS 2는 기본적으로 노드를 탐색할 때 멀티캐스트를 사용합니다. 
   * 하지만 학교 유선랜이나 공용 Wi-Fi 공유기는 보안 및 트래픽 과부하 방지를 위해 **멀티캐스트 패킷을 강제로 필터링/차단**합니다. 따라서 노트북과 로봇이 물리적으로 같은 유선랜에 꽂혀 있어도 ROS 2 노드를 서로 발견하지 못합니다.

---

## 2. 해결 시나리오별 설정법

### 💡 시나리오 A: 동일한 물리 포트에 두 IP를 얹은 경우 (`set_both.sh` IP Aliasing 방식)
> **하드웨어 셋업**: Jetson의 단일 `eth0` 포트가 스위칭 허브를 거쳐 로봇 본체와 학교 유선랜 벽면 포트에 동시에 물려있는 상황.

이 경우 물리 카드(`eth0`)는 하나이지만 대역이 2개(`192.168.123.xx` 및 `203.252.107.xx`)입니다. 이더넷 드라이버 상에서는 `eth0`로 단일 바인딩되나, **학교망 스위치의 멀티캐스트 차단** 때문에 노트북과 Jetson이 통신하지 못합니다.

#### 🛠️ 해결책: 유니캐스트 피어(`Peers`) 강제 등록
`cyclonedds.xml` 파일에 외부 노트북의 IP와 Jetson의 IP를 직접 지정하여 멀티캐스트 없이 **1:1 유니캐스트로 통신**하도록 뚫어줍니다.

**[cyclonedds.xml 설정]**:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <!-- 단일 포트 Aliasing인 경우 eth0 바인딩만으로 두 대역 통신 가능 -->
            <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
            <AllowMulticast>default</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer Address="203.252.107.219" /> <!-- Jetson 외부망 IP -->
                <Peer Address="203.252.107.XXX" /> <!-- 노트북 외부망 IP (실제 할당받은 IP로 변경) -->
            </Peers>
        </Discovery>
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
    </Domain>
</CycloneDDS>
```

---

### 💡 시나리오 B: 서로 다른 2개의 물리 포트를 사용하는 경우 (Multi-NIC 방식)
> **하드웨어 셋업**: 
> * **`eth0` (Jetson 기본 유선포트)**: 로봇 본체와 1:1 직접 연결 (`192.168.123.xx`)
> * **`eth1` (USB to Ethernet 랜카드)** 또는 **`wlan0` (Wi-Fi)**: 학교 유선랜 콘센트 또는 무선 공유기에 연결 (`203.252.107.xx`)

이 방식은 패킷 간섭이 없고 안정적이어서 실제 주행 셋업에서 가장 선호되는 방식입니다. CycloneDDS가 두 카드를 모두 다루도록 열어주어야 합니다.

#### 🛠️ 해결책: `<Interfaces>` 다중 인터페이스 바인딩
CycloneDDS가 로봇망 포트와 외부망 포트를 둘 다 리스닝하고 통신 경로를 잡을 수 있도록 명시적으로 가동 카드를 나열합니다.

**[cyclonedds.xml 설정]**:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
             xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <!-- 중요: CycloneDDS가 감시할 네트워크 카드 목록을 모두 적어줍니다. -->
            <Interfaces>
                <NetworkInterface name="eth0" />   <!-- 로봇 본체 연결용 내부망 -->
                <NetworkInterface name="eth1" />   <!-- 노트북 연결용 외부망 (무선인 경우 wlan0) -->
            </Interfaces>
            <AllowMulticast>default</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer Address="203.252.107.219" /> <!-- Jetson 외부망 IP -->
                <Peer Address="203.252.107.XXX" /> <!-- 노트북 외부망 IP (실제 할당받은 IP로 변경) -->
            </Peers>
        </Discovery>
        <Internal>
            <SocketReceiveBufferSize min="10MB"/>
        </Internal>
    </Domain>
</CycloneDDS>
```

---

## 3. 통신 검증 절차 (Verification Steps)

DDS 라우팅 셋업 완료 후 노트북과 로봇 간에 정상 통신이 되는지 확인하는 가장 빠르고 확실한 순서입니다.

1. **핑(Ping) 테스트**:
   * 노트북 ➔ Jetson 외부망 IP (`ping 203.252.107.219`)
   * Jetson ➔ 로봇 본체 IP (`ping 192.168.123.161`)
2. **토픽 발행 검증 (Jetson ➔ 노트북)**:
   * **Jetson**: 임시로 아무 토픽이나 발행합니다.
     ```bash
     ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello From Jetson'"
     ```
   * **노트북**: 동일한 CycloneDDS 설정을 켠 후 토픽을 확인합니다.
     ```bash
     ros2 topic echo /test_topic
     ```
3. **토픽 발행 검증 (로봇 본체 ➔ 노트북)**:
   * Jetson에서 `run_map_indoor.sh`를 실행하여 로봇 본체의 오도메트리 토픽(`/odom`)이 활성화된 상태에서, 노트북 터미널에 `ros2 topic hz /odom`을 쳐서 본체 모터 데이터가 다이렉트로 흘러오는지 최종 점검합니다.
